# CdacUsageGraph

A standalone Roslyn analysis tool that extracts the **contract -> Data-type usage
graph** for the cDAC (contract-based Data Access Component) and compares it against
the hand-written `docs/design/datacontracts/*.md` data-descriptor tables.

For each registered contract (and version), it reports which `Data.*` structs and
which of their `[Field]` descriptor fields the contract implementation uses.

## What it does

1. Builds a `CSharpCompilation` over the cDAC `Abstractions` + `Contracts` source
   (analyzing the source directly -- the cDAC projects don't need to be compiled first,
   and the source-generated `IData<T>.Create` factories are not needed to analyze usage;
   reference assemblies come from the tool's own runtime).
2. Parses `CoreCLRContracts.Register` to map `(interface, version) -> impl type`.
3. Discovers `Data.*` types (`[CdacType]` / `IData<T>`) and their
   `[Field]`/`[FieldAddress]` properties.
4. Performs a **forward interprocedural walk** from each contract impl's members
   (methods, constructors, and **field/property initializers**), propagating a
   `(contract, version)` label through:
   - helper method calls (into the same assembly),
   - constructed helper objects (`new FrameHelpers(...)`), including those built in
     field initializers (e.g. `StressLog_1`'s `new SmallStressMessageReader(...)`),
   - base/generic-base classes (with type-parameter substitution),
   - virtual dispatch into nested classes,
   - reads through a shared Data-implemented interface (e.g. an `IExceptionClauseData`
     or `IGCHeap` local that may hold one of several Data types at runtime -- each
     implementing Data type's `[Field]` members are conservatively attributed), and
   - static-abstract dispatch through a generic type parameter
     (`TImpl.StubPrecode_GetMethodDesc(...)`).
5. Collects usage in one `OperationWalker` pass into a single per-`(contract, version,
   Data type)` field map (the set of Data types used is derivable from its keys):
   - `GetOrAdd<Data.X>()` / generic type arguments / `new Data.X(...)` / `typeof` --
     recorded as a type usage (an entry with no field if nothing is read),
   - property value reads (`Read` / `Write` / `ReadWrite`),
   - `nameof(Data.X.Field)` and raw-string `TypeInfo.Fields["nativeName"]` offset
     lookups (`OffsetLookup`), and
   - `TypeInfo.Size` reads, recorded as a synthetic `Size` field.

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
│       ├── Compilation/               # CdacCompilationLoader (phase A)
│       ├── Discovery/                 # DataTypeIndex, ContractRegistrationParser, TypeInfoCorrelator (phase B)
│       ├── Analysis/                  # UsageWalker (OperationWalker), UsageCollector, OperationInspector (phase C/D)
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
| `contract-contracts-used.md`  | `(contract, version) -> other contracts used` (`_target.Contracts.<X>`) |
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

The generation logic lives in `Docs/DocGenerator.cs` so the CI unit test and the manual
regen use one implementation. The loader derives its source list (including the coreclr tool
files linked via
`<Compile Include>`) directly from the cDAC `.csproj`s and **fails fast** if a referenced linked
file is missing or if discovery finds no Data types / registrations -- so a broken or drifted
compilation input surfaces as an error rather than a silently under-reported graph.

## Known limitations

- **Over-approximation (conservative by design).** The walk seeds from *all* of a contract
  implementation's methods and follows *all* methods of the helpers/base types it constructs, so a
  descriptor is reported if it is reachable on *any* path -- including paths that are never actually
  taken at runtime. The tables answer "what could this contract read", not precise reachability.
  Use the `_suppress` list in `data-descriptor-meanings.json` to prune specific false positives.
- Field access through **indirect `TypeInfo` flows** (a `Target.TypeInfo` returned
  from a helper method, or stored in a collection) is not traced.
- **Interface-typed reads are conservative.** When a contract reads a member through
  an interface implemented by several Data types (e.g. `IExceptionClauseData`,
  `IGCHeap`), the read is attributed to the `[Field]` member of *every* implementing
  Data type -- the concrete runtime type can't be known statically. Members that a
  given type implements as computed/pass-through (non-`[Field]`) properties are
  correctly not credited to that type.
- **Delegate / `Func<>`** call edges are not resolved (as with any static analysis).
- **Cross-contract** dependencies are correctly *not* attributed to the caller: a
  contract that calls `_target.Contracts.<X>` records `X` in its **Contracts used**
  list rather than absorbing `X`'s data descriptors.


