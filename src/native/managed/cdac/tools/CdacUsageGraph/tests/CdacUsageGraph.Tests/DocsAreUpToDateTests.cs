// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph;
using CdacUsageGraph.Docs;
using CdacUsageGraph.Model;
using Xunit;

namespace CdacUsageGraph.Tests;

/// <summary>
/// Drift gate: the generated marker blocks in <c>docs/design/datacontracts/*.md</c> must match what
/// the analysis currently produces. This runs the same <see cref="DocGenerator"/> the <c>docs</c>
/// command uses; if it fails, run <c>CdacUsageGraph docs</c> (or <c>generate-docs.ps1</c>) and commit
/// the result. Skipped when the cDAC source can't be located (e.g. running outside the repo).
/// </summary>
public sealed class DocsAreUpToDateTests
{
    [Fact]
    public void GeneratedDocBlocksMatchAnalysis()
    {
        DirectoryInfo? cdacRoot = Locator.FindCdacRoot();
        if (cdacRoot is null) return; // cDAC source not found (running outside the repo)

        DirectoryInfo docsDir = Locator.DocsDirectory(cdacRoot);
        if (!docsDir.Exists) return;

        UsageGraph graph = AnalysisPipeline.BuildGraph(cdacRoot.FullName);
        DocDescriptorMeanings meanings = DocDescriptorMeanings.Load(Locator.MeaningsFile(cdacRoot).FullName);
        DocGenerator generator = new DocGenerator(graph, meanings);

        IReadOnlyList<string> drifted = generator.Check(docsDir.FullName);

        Assert.True(
            drifted.Count == 0,
            $"The generated data-descriptor doc blocks are out of date for: {string.Join(", ", drifted)}. " +
            "Run 'CdacUsageGraph docs' (or generate-docs.ps1) and commit the result.");
    }
}
