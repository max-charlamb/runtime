// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Xml.Linq;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace CdacUsageGraph.Compilation;

/// <summary>
/// Phase A: builds a <see cref="CSharpCompilation"/> over the cDAC Abstractions + Contracts
/// source. Each project's own source is globbed from its directory, and the extra source files
/// the project links in via explicit <c>&lt;Compile Include="..."/&gt;</c> items (e.g. the coreclr
/// tool files) are read straight from the <c>.csproj</c> so the input can't silently drift.
/// No MSBuild restore is required; the source-generated <c>IData&lt;T&gt;.Create</c> factories are
/// not needed to analyze contract-to-Data usage, so compilation errors are expected and non-fatal.
/// </summary>
public sealed class CdacCompilationLoader
{
    // The manual compilation intentionally omits the IData source generator. Keep the expected
    // error surface explicit: a new/missing diagnostic means the semantic model changed and the
    // analysis could silently under-report, so CI should require this baseline to be reviewed.
    private static readonly Dictionary<string, int> s_expectedErrorCounts =
        new Dictionary<string, int>(StringComparer.Ordinal)
        {
            ["CS0103"] = 6,   // Generated Write<Property> method is not in the current context.
            ["CS0117"] = 1,   // Generated static Data member is absent.
            ["CS0535"] = 188, // IData<T>.Create implementations are generated.
            ["CS0759"] = 29,  // OnInit partial definitions are generated.
            ["CS1061"] = 45,  // Generated members/constructors are absent.
            ["CS1729"] = 4,   // Generated Data constructors are absent.
            ["CS8795"] = 3,   // Required partial method implementations are generated.
        };

    private static readonly string[] s_projects =
    [
        "Microsoft.Diagnostics.DataContractReader.Abstractions",
        "Microsoft.Diagnostics.DataContractReader.Contracts",
    ];

    /// <summary>Loads and parses the cDAC source rooted at <paramref name="cdacRoot"/>.</summary>
    public CSharpCompilation Load(string cdacRoot)
    {
        List<string> files = new List<string>();
        List<string> missingLinked = new List<string>();

        foreach (string project in s_projects)
        {
            string dir = Path.Combine(cdacRoot, project);

            // 1. The project's own (SDK-globbed) source.
            foreach (string f in Directory.EnumerateFiles(dir, "*.cs", SearchOption.AllDirectories))
            {
                if (f.Contains($"{Path.DirectorySeparatorChar}obj{Path.DirectorySeparatorChar}") ||
                    f.Contains($"{Path.DirectorySeparatorChar}bin{Path.DirectorySeparatorChar}"))
                    continue;
                files.Add(f);
            }

            // 2. Files linked in via explicit <Compile Include="..."/> (e.g. coreclr tool files).
            //    Reading these from the .csproj keeps the compilation input in lock-step with the
            //    real project instead of a hand-maintained list that can silently drift.
            string csproj = Path.Combine(dir, project + ".csproj");
            foreach (string include in ReadExplicitCompileIncludes(csproj))
            {
                string resolved = Path.GetFullPath(Path.Combine(dir, include.Replace('\\', Path.DirectorySeparatorChar)));
                if (File.Exists(resolved))
                    files.Add(resolved);
                else
                    missingLinked.Add(include);
            }
        }

        // Staleness guard: if the .csproj references linked files that no longer exist, the walk
        // would quietly under-report. Fail loudly instead so the tool (and its CI drift gate) is
        // trustworthy.
        if (missingLinked.Count > 0)
        {
            throw new InvalidOperationException(
                "The cDAC project(s) reference linked source files that no longer exist; the tool's " +
                "compilation input is stale. Update CdacCompilationLoader/the source list. Missing:" +
                Environment.NewLine + "  " + string.Join(Environment.NewLine + "  ", missingLinked));
        }

        CSharpParseOptions parseOptions = new CSharpParseOptions(LanguageVersion.Preview);
        List<SyntaxTree> trees = files
            .Distinct()
            .Select(f => CSharpSyntaxTree.ParseText(File.ReadAllText(f), parseOptions, path: f))
            .ToList();

        CSharpCompilation compilation = CSharpCompilation.Create(
            "CdacUsageAnalysis",
            trees,
            GetRuntimeReferences(),
            new CSharpCompilationOptions(
                OutputKind.DynamicallyLinkedLibrary,
                allowUnsafe: true,
                nullableContextOptions: NullableContextOptions.Enable));
        ValidateExpectedDiagnostics(compilation);
        return compilation;
    }

    private static void ValidateExpectedDiagnostics(CSharpCompilation compilation)
    {
        Dictionary<string, int> actual = compilation.GetDiagnostics()
            .Where(d => d.Severity == DiagnosticSeverity.Error)
            .GroupBy(d => d.Id)
            .ToDictionary(g => g.Key, g => g.Count(), StringComparer.Ordinal);

        if (actual.Count == s_expectedErrorCounts.Count &&
            actual.All(kv => s_expectedErrorCounts.TryGetValue(kv.Key, out int expected) && expected == kv.Value))
            return;

        static string Format(IEnumerable<KeyValuePair<string, int>> counts) =>
            string.Join(", ", counts.OrderBy(kv => kv.Key, StringComparer.Ordinal).Select(kv => $"{kv.Key}={kv.Value}"));

        throw new InvalidOperationException(
            "The cDAC analysis compilation diagnostic baseline changed. Because compilation errors " +
            "can degrade Roslyn operations and silently under-report usage, review the diagnostics " +
            "and update CdacCompilationLoader only if the change is expected." + Environment.NewLine +
            "Expected: " + Format(s_expectedErrorCounts) + Environment.NewLine +
            "Actual:   " + Format(actual));
    }

    // Reference assemblies for the compilation: the trusted-platform-assembly set of the runtime
    // the tool is executing on ($(NetCoreAppToolCurrent)). This is a superset of what the cDAC
    // source needs (System.*), so the semantic model resolves BCL types without pulling in the
    // separate Basic.Reference.Assemblies package. The source-generated IData<T>.Create factories
    // are still absent, so a handful of compile errors remain expected and non-fatal.
    private static IEnumerable<MetadataReference> GetRuntimeReferences()
    {
        if (AppContext.GetData("TRUSTED_PLATFORM_ASSEMBLIES") is not string tpa)
            throw new InvalidOperationException("Could not enumerate runtime reference assemblies (TRUSTED_PLATFORM_ASSEMBLIES).");

        foreach (string path in tpa.Split(Path.PathSeparator))
        {
            if (path.EndsWith(".dll", StringComparison.OrdinalIgnoreCase) && File.Exists(path))
                yield return MetadataReference.CreateFromFile(path);
        }
    }

    // Explicit single-file <Compile Include="..."/> items from a .csproj (SDK-style projects only
    // list the linked/external files explicitly; their own source is globbed implicitly). Glob
    // patterns are skipped -- only literal file references are treated as linked inputs.
    private static IEnumerable<string> ReadExplicitCompileIncludes(string csprojPath)
    {
        if (!File.Exists(csprojPath))
            yield break;

        XDocument doc;
        try
        {
            doc = XDocument.Load(csprojPath);
        }
        catch (System.Xml.XmlException)
        {
            yield break;
        }

        foreach (XElement element in doc.Descendants().Where(e => e.Name.LocalName == "Compile"))
        {
            string? include = element.Attribute("Include")?.Value;
            if (!string.IsNullOrEmpty(include) && !include.Contains('*'))
                yield return include;
        }
    }
}
