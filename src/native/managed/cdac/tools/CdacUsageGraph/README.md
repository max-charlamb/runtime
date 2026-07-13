# CdacUsageGraph

A standalone Roslyn analysis tool that extracts the **contract -> Data-type usage
graph** for the cDAC (contract-based Data Access Component) and compares it against
the hand-written `docs/design/datacontracts/*.md` data-descriptor tables.

For each registered contract (and version), it reports which `Data.*` structs and
which of their `[Field]` descriptor fields the contract implementation uses.

## What it does

1. Builds a `CSharpCompilation` over the cDAC `Abstractions` + `Contracts` source
   (no MSBuild/Arcade restore required; the source-generated `IData<T>.Create`
   factories are not needed to analyze usage).
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

Requires a .NET SDK that can target `net8.0` (the tool is isolated from the
runtime's Arcade build via the local `Directory.Build.props/.targets` and
`Directory.Packages.props`).

```powershell
# from this folder (the solution root)
dotnet run --project src/CdacUsageGraph -c Release
```

Optional arguments: `--cdac-root <dir>` (the cDAC source root; auto-detected if
omitted) and `--output <dir>` (defaults to this folder's `output/`).

### Project layout

```
CdacUsageGraph/                        # solution root (isolated from the Arcade build)
├── CdacUsageGraph.slnx
├── src/
│   └── CdacUsageGraph/                # the tool (Exe): thin Program.cs + all analysis logic
│       ├── AnalysisOptions.cs, AnalysisPipeline.cs, Commands.cs, Locator.cs, Program.cs
│       ├── Compilation/               # CdacCompilationLoader (phase A)
│       ├── Discovery/                 # DataTypeIndex, ContractRegistrationParser, TypeInfoCorrelator (phase B)
│       ├── Analysis/                  # UsageWalker (OperationWalker), UsageCollector, OperationInspector (phase C/D)
│       ├── Model/                     # UsageGraph, RegistrationInfo (immutable result)
│       └── Reporting/                 # IReportWriter + Markdown/JSON writers (phase E)
└── tests/
    └── CdacUsageGraph.Tests/          # xUnit: in-memory-compilation + end-to-end tests
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
dotnet test tests/CdacUsageGraph.Tests -c Release
```

### Compare against the docs

```powershell
pwsh ./compare.ps1
```

Writes `./output/doc-comparison.md` with type-level and field-level diffs
(`DocOnly` / `ToolOnly`). Names are normalized before diffing (type aliases such as
`GCHeapSVR`=`GCHeap`, trailing `_<version>` stripped, and field names compared
without `m_`/`_` prefixes), so only genuine drift is surfaced.

### Running as a CI drift gate

The tool is isolated from the Arcade build, so nothing builds or restores it in the normal
runtime build; the drift gate must restore + run it explicitly. It depends only on in-feed
packages (Roslyn, `Basic.Reference.Assemblies`, `System.CommandLine`), which restore during
`dotnet build`. A self-contained gate is:

```powershell
dotnet build  src/native/managed/cdac/tools/CdacUsageGraph/CdacUsageGraph.slnx -c Release
dotnet run    --project src/native/managed/cdac/tools/CdacUsageGraph/src/CdacUsageGraph -c Release
pwsh          src/native/managed/cdac/tools/CdacUsageGraph/generate-docs.ps1 -Check   # fails on drift
```

The loader derives its source list (including the coreclr tool files linked via
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


