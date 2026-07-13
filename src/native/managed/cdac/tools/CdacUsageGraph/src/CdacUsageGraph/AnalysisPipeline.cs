// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis;
using CdacUsageGraph.Compilation;
using CdacUsageGraph.Discovery;
using CdacUsageGraph.Model;
using CdacUsageGraph.Reporting;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace CdacUsageGraph;

/// <summary>
/// Orchestrates the analysis: load compilation (A) -&gt; discover Data types and registrations (B)
/// -&gt; forward interprocedural walk (C/D) -&gt; emit reports (E).
/// </summary>
public sealed class AnalysisPipeline
{
    private static readonly IReportWriter[] s_writers =
    [
        new DataGraphMarkdownWriter(),
        new FieldUsageMarkdownWriter(),
        new ContractsUsedMarkdownWriter(),
        new JsonReportWriter(),
    ];

    private readonly AnalysisOptions _options;

    public AnalysisPipeline(AnalysisOptions options) => _options = options;

    public int Run()
    {
        string cdacRoot = _options.CdacRoot.FullName;
        if (!Directory.Exists(Path.Combine(cdacRoot, "Microsoft.Diagnostics.DataContractReader.Contracts")))
            throw new InvalidOperationException($"Could not find the cDAC Contracts project under '{cdacRoot}'; pass --cdac-root.");

        // Phase A: compilation.
        CSharpCompilation compilation = new CdacCompilationLoader().Load(cdacRoot);
        int errCount = compilation.GetDiagnostics().Count(d => d.Severity == DiagnosticSeverity.Error);
        Console.WriteLine($"Parsed {compilation.SyntaxTrees.Length} files. Compilation errors (expected, non-fatal): {errCount}");

        // Phase B: discovery.
        DataTypeIndex index = DataTypeIndex.Build(compilation);
        Console.WriteLine($"Discovered {index.Count} Data types.");

        IReadOnlyList<ContractRegistration> registrations = ContractRegistrationParser.Parse(compilation);
        Console.WriteLine($"Parsed {registrations.Count} contract registrations.");

        // Sanity guard: if discovery found no Data types or no registrations, the compilation
        // input has drifted (renamed anchor types, missing source) -- fail fast rather than emit
        // an empty/misleading graph.
        if (index.Count == 0 || registrations.Count == 0)
            throw new InvalidOperationException(
                $"Sanity check failed: discovered {index.Count} Data types and {registrations.Count} " +
                "contract registrations. The cDAC compilation input is likely broken or has drifted.");

        TypeInfoCorrelator correlator = TypeInfoCorrelator.Build(compilation);

        // Phase C/D: forward interprocedural walk.
        UsageGraph graph = new UsageWalker(compilation, index, correlator).Walk(registrations, cdacRoot);

        // Phase E: emit.
        string outDir = _options.OutputDirectory.FullName;
        Directory.CreateDirectory(outDir);
        Console.WriteLine($"Wrote outputs to {outDir}");
        foreach (IReportWriter writer in s_writers)
            Console.WriteLine("  " + writer.Write(graph, outDir));

        return 0;
    }
}
