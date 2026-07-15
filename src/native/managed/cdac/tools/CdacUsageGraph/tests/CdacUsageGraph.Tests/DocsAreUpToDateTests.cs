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

    [Theory]
    [InlineData("<!-- BEGIN GENERATED: data-descriptors contract=Thread version=c1 -->")]
    [InlineData(
        "<!-- BEGIN GENERATED: unknown contract=Thread version=c1 -->\n" +
        "<!-- END GENERATED: unknown contract=Thread version=c1 -->")]
    [InlineData(
        "<!-- BEGIN GENERATED: contracts-used contract=Thread version=c1 -->\n" +
        "<!-- END GENERATED: contracts-used contract=Thread version=c1 -->\n" +
        "<!-- BEGIN GENERATED: contracts-used contract=Thread version=c1 -->\n" +
        "<!-- END GENERATED: contracts-used contract=Thread version=c1 -->")]
    public void RejectsInvalidGeneratedMarkers(string content)
    {
        using TempDirectory temp = new();
        File.WriteAllText(Path.Combine(temp.Path, "Thread.md"), content);
        DocGenerator generator = new(EmptyGraph(), DocDescriptorMeanings.Empty);

        Assert.Throws<InvalidOperationException>(() => generator.Check(temp.Path));
    }

    [Fact]
    public void RejectsMalformedDescriptorOverrideKey()
    {
        using TempDirectory temp = new();
        string path = Path.Combine(temp.Path, "meanings.json");
        File.WriteAllText(path, """{ "_supplement": { "Thread": ["MissingDot"] } }""");

        Assert.Throws<System.Text.Json.JsonException>(() => DocDescriptorMeanings.Load(path));
    }

    [Fact]
    public void GeneratesManagedCdacNamesWithoutSplittingAtNamespaceDots()
    {
        using TempDirectory temp = new();
        string path = Path.Combine(temp.Path, "Thread.md");
        File.WriteAllText(path,
            "<!-- BEGIN GENERATED: data-descriptors contract=Thread version=c1 -->\n" +
            "<!-- END GENERATED: data-descriptors contract=Thread version=c1 -->");
        UsageGraph graph = new(
            "",
            1,
            [],
            new Dictionary<(ContractLabel, string), IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>>
            {
                [(new ContractLabel("IThread", "c1"), "Data.System.Threading.Lock")] =
                    new Dictionary<string, IReadOnlyCollection<UsageKind>>
                    {
                        ["_state"] = new[] { UsageKind.Read },
                    },
            },
            new Dictionary<ContractLabel, IReadOnlyCollection<string>>());

        new DocGenerator(graph, DocDescriptorMeanings.Empty).Emit(temp.Path);

        string generated = File.ReadAllText(path);
        Assert.Contains("| `System.Threading.Lock` | `_state` | _TODO: describe_ |", generated);
    }

    private static UsageGraph EmptyGraph() => new(
        "",
        0,
        [],
        new Dictionary<(ContractLabel, string), IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>>(),
        new Dictionary<ContractLabel, IReadOnlyCollection<string>>());

    private sealed class TempDirectory : IDisposable
    {
        public TempDirectory() => Path = Directory.CreateTempSubdirectory("CdacUsageGraphTests").FullName;

        public string Path { get; }

        public void Dispose() => Directory.Delete(Path, recursive: true);
    }
}
