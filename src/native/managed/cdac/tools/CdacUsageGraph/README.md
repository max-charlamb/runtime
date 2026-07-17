# CdacUsageGraph

A standalone Roslyn analysis tool that extracts the **contract -> Data-type usage
graph** for the cDAC (contract-based Data Access Component) and compares it against
the hand-written `docs/design/datacontracts/*.md` data-descriptor tables.

For each registered contract (and version), it reports which `Data.*` structs and
which of their `[Field]` descriptor fields the contract implementation uses.

## What it does

1. Opens the real cDAC `Microsoft.Diagnostics.DataContractReader.Contracts.csproj`
   in the Debug configuration through Roslyn `MSBuildWorkspace` and obtains its
   generated `CSharpCompilation`.
   This preserves evaluated compile items, linked files, project references, build
   properties, references, analyzer configuration, and the real
   `Microsoft.Diagnostics.DataContractReader.DataGenerator` output. The analysis
   therefore includes the same generated Data constructors, `IData<T>.Create`
   factories, `Write<Property>` methods and helper types as the product build.
2. Parses `CoreCLRContracts.Register` to map `(interface, version) -> impl type`.
3. Discovers real `IData<TSelf>` Data types and their
   `[Field]`/`[FieldAddress]` properties.
4. Performs a **forward interprocedural specialization walk** from each registered
   contract interface implementation, its selected constructor, and
   **field/property initializers**, propagating a `(contract, version)` label through:
   - helper method calls (into the same assembly),
   - constructed helper objects (`new FrameHelpers(...)`), including those built in
     field initializers (e.g. `StressLog_1`'s `new SmallStressMessageReader(...)`),
   - method-group callbacks passed across contract boundaries,
   - base/generic-base classes (with type-parameter substitution),
   - virtual/interface dispatch over the contract's reachable constructed types,
   - reads through a shared Data-implemented interface (e.g. an `IExceptionClauseData`
     or `IGCHeap` local that may hold one of several Data types at runtime -- each
     implementing Data type's `[Field]` members are conservatively attributed), and
   - static-abstract dispatch through a generic type parameter
     (`TImpl.StubPrecode_GetMethodDesc(...)`).
   Generic contract bases are specialized per registered version, so static-abstract
   and virtual calls reach only that version's concrete implementation.
5. Collects usage into a single per-`(contract, version, Data type)` field map (the set
   of Data types used is derivable from its keys). `UsageWalker` owns contract reachability
   and Data-property usage, while the CFG analysis contributes all `TypeInfo`-derived field
   effects:
   - `GetOrAdd<Data.X>()` / generic type arguments / `new Data.X(...)` / `typeof` --
     recorded as a type usage (an entry with no field if nothing is read),
   - property value reads (`Read` / `Write` / `ReadWrite`). Reads of a **computed**
     Data property (one with a getter body, e.g. `Assembly.IsError => Error != Null`
     or `TLSIndex.IndexOffset => TLSIndexRawIndex & 0xFFFFFF`) are resolved to the
     actual descriptor fields the getter reads, rather than the derived name.
     `[Field]` auto-properties are recorded directly; properties populated by
     `OnInit` (e.g. `Thread.ThreadHandle`) expand to the descriptor reads in
     the initializer,
   - `nameof(Data.X.Field)` and raw-string `TypeInfo.Fields["nativeName"]` offset
     lookups (`OffsetLookup`), and
   - `TypeInfo.Size` reads, recorded as a synthetic `Size` field.
   Global reads through `ReadGlobal*` / `TryReadGlobal*` are collected with their
   resolved name, native type, and required/optional access. Constant names propagate
   through helper parameters and switch expressions; enum-derived name families are
   represented symbolically (for example, `<FrameType>Identifier`).
   `Target.TypeInfo` identities are propagated by a Roslyn ControlFlowGraph analysis
   through assignments, branch joins, loops, method returns, `out`/`ref` parameters,
   method/constructor arguments, fields, and interface-to-implementation parameters,
   so
   `Read*Field(..., typeInfo, nameof(Field))` and `typeInfo.Fields[nameof(Field)]`
   inside helpers such as `DacEnumerableHash` are reduced to field-access effects and
   attributed to each concrete Data type by `UsageWalker`. The walker does not maintain
   a separate name-based `TypeInfo` correlator or helper-method summaries.
   Parsed aggregate properties assigned by `OnInit`
   (e.g. `EETypeHashTable.Entries`) are replaced by those actual underlying fields.
   Constructor-derived aggregates (e.g. Loader's `DynamicILBlobTable.HashTable`) are
   handled similarly. Each Data type has one ordered cDAC name set from
   `[CdacType]`: the first name is used in reports and all names are available for
   layout lookup. This handles adapter names (`DynamicILBlobEntry` ->
   `DynamicILBlobTable`) and managed layout names without requiring a `DataType`
   enum element.

## Build & run

The tool is part of the runtime's Arcade build (subset `tools.cdactests`), targeting
`$(NetCoreAppToolCurrent)` and using the repo-central Roslyn / `System.CommandLine`
versions. Build/test it with the repo build:

```bash
./build.sh -s tools.cdac+tools.cdactests -c Debug -test     # build.cmd on Windows
```

For a quick local run of the analysis (using the repo SDK under `.dotnet`):

```powershell
dotnet run --project src/CdacUsageGraph -c Debug
```

Optional arguments: `--cdac-root <dir>` (the cDAC source root; auto-detected if
omitted) and `--output <dir>` (defaults to this folder's `output/`).

The tool also has a `docs` sub-command that fills the generated marker blocks in
`docs/design/datacontracts/*.md` (see below).

### Project layout

```
CdacUsageGraph/                        # tool root (part of the Arcade build)
├── CdacUsageGraph.slnx
├── src/
│   └── CdacUsageGraph/                # the tool (Exe): thin Program.cs + all analysis logic
│       ├── AnalysisOptions.cs, AnalysisPipeline.cs, Commands.cs, Locator.cs, Program.cs
│       ├── Compilation/               # CdacWorkspaceLoader / MSBuildWorkspace (phase A)
│       ├── Discovery/                 # DataTypeInfo/Index, ContractRegistrationParser (phase B)
│       ├── Analysis/                  # UsageWalker plus CFG DataFlow effects (phase C/D)
│       ├── Model/                     # UsageGraph, RegistrationInfo (immutable result)
│       ├── Reporting/                 # IReportWriter + Markdown/JSON writers (phase E)
│       └── Docs/                      # DocGenerator + DocDescriptorMeanings (fills the docs marker blocks)
└── tests/
    └── CdacUsageGraph.Tests/          # xUnit: in-memory-compilation, end-to-end, and doc-drift tests
```

The analysis logic lives in the single `CdacUsageGraph` project (folders are namespaces); the
test project references it via `<ProjectReference>` + `InternalsVisibleTo`.

### Outputs (written to `./output`)

| File | Contents |
|------|----------|
| `contract-data-graph.md`      | `(contract, version) -> Data types` table |
| `contract-field-usage.md`     | `(contract, version, Data.Type, field) -> usage kind` rows |
| `contract-global-usage.md`    | `(contract, version, global) -> native type and required/optional access` |
| `contract-contracts-used.md`  | `(contract, version) -> other contracts used` (`_target.Contracts.<X>`) |
| `contract-methods-reachable.md` | `(contract, version) -> reachable specialized method contexts` |
| `contract-usage.json`         | Machine-readable graph |

### Tests

```powershell
dotnet test tests/CdacUsageGraph.Tests -c Debug
```

Included tests cover discovery, the end-to-end walk against the real cDAC source, and the
**doc-drift gate** (`DocsAreUpToDateTests`) which asserts the generated marker blocks in
`docs/design/datacontracts/*.md` are up to date. Because the test project is in the
`tools.cdactests` subset, this gate runs in the `CdacUnitTests` CI leg on every PR that
touches `src/native/managed/cdac/**`.

### Compare against the docs

```powershell
pwsh ./compare.ps1
```

Writes `./output/doc-comparison.md` with type-level and field-level diffs
(`DocOnly` / `ToolOnly`). Names are normalized before diffing (type aliases such as
`GCHeapSVR`=`GCHeap`, trailing `_<version>` stripped, and field names compared
without `m_`/`_` prefixes), so only genuine drift is surfaced.

### Generating / checking the docs

The `docs` sub-command (and its thin `generate-docs.ps1` wrapper) fills the generated marker
blocks from the analysis, merging in `data-descriptor-meanings.json`:

```powershell
pwsh ./generate-docs.ps1           # rewrite marked blocks in place
pwsh ./generate-docs.ps1 -Check    # fail on drift (same logic as the CI unit test)
```

`usage` is the only supported marker kind. Each block contains alphabetized tables for data
descriptors, globals, and contracts used. Descriptor rows are sorted by descriptor name and then
field name; the sidecar supplies human-authored meanings while names and native types come from
the analysis.

The documentation generation logic lives in `Docs/DocGenerator.cs` so the CI unit test and the
regen use one implementation. `MSBuildWorkspace` evaluates the real Contracts project, including
its linked CoreCLR source files and generator analyzer reference. Workspace load failures,
compilation errors, or missing Data types / registrations fail immediately rather than producing
a silently under-reported graph.

## Known limitations

- **Conservative method bodies.** Every operation in a reachable method body is considered.
  Version-specific capabilities should therefore be represented by version-specific methods
  rather than conditions inside a shared implementation.
- `TypeInfo` values stored inside arbitrary **collections/arrays** are not tracked.
  Locals, parameters, returns, `out`/`ref`, fields, calls, branches, and loops are
  flow-analyzed.
- **Interface-typed reads are conservative.** When a contract reads a member through
  an interface implemented by several Data types (e.g. `IExceptionClauseData`,
  `IGCHeap`), the read is attributed to *every* implementing Data type -- the concrete
  runtime type can't be known statically. Each implementation uses the same property
  provenance rules as a direct read, so computed/parsed properties resolve to their
  actual underlying fields.
- Reachable method-group callbacks and inline anonymous-function bodies are analyzed. Delegate
  values flowing through arbitrary fields, collections, or external factories remain conservative.
- **Cross-contract** dependencies are correctly *not* attributed to the caller: a
  contract that calls `_target.Contracts.<X>` records `X` in its **Contracts used**
  list rather than absorbing `X`'s data descriptors.
