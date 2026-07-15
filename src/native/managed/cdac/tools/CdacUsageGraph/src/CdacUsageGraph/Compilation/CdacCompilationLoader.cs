// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Xml.Linq;
using Microsoft.Diagnostics.DataContractReader.DataGenerator;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace CdacUsageGraph.Compilation;

/// <summary>
/// Phase A: builds a <see cref="CSharpCompilation"/> over the cDAC Abstractions + Contracts
/// source. Each project's own source is globbed from its directory, and the extra source files
/// the project links in via explicit <c>&lt;Compile Include="..."/&gt;</c> items (e.g. the coreclr
/// tool files) are read straight from the <c>.csproj</c> so the input can't silently drift.
/// The real cDAC <see cref="CdacGenerator"/> is then run through a Roslyn
/// <see cref="GeneratorDriver"/>, producing the same Data constructors, factories, writable-field
/// methods and helper types used by the product build. The resulting compilation must be error-free.
/// </summary>
internal static class CdacCompilationLoader
{
    private static readonly string[] s_projects =
    [
        "Microsoft.Diagnostics.DataContractReader.Abstractions",
        "Microsoft.Diagnostics.DataContractReader.Contracts",
    ];

    /// <summary>Loads and parses the cDAC source rooted at <paramref name="cdacRoot"/>.</summary>
    public static CSharpCompilation Load(string cdacRoot)
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

        CSharpCompilation inputCompilation = CSharpCompilation.Create(
            "CdacUsageAnalysis",
            trees,
            GetRuntimeReferences(),
            new CSharpCompilationOptions(
                OutputKind.DynamicallyLinkedLibrary,
                allowUnsafe: true,
                nullableContextOptions: NullableContextOptions.Enable));

        ISourceGenerator generator = new CdacGenerator().AsSourceGenerator();
        GeneratorDriver driver = CSharpGeneratorDriver.Create(
            generators: [generator],
            parseOptions: parseOptions);
        driver.RunGeneratorsAndUpdateCompilation(
            inputCompilation,
            out Microsoft.CodeAnalysis.Compilation outputCompilation,
            out System.Collections.Immutable.ImmutableArray<Diagnostic> generatorDiagnostics);

        if (outputCompilation is not CSharpCompilation compilation)
            throw new InvalidOperationException("The cDAC generator did not produce a C# compilation.");

        List<Diagnostic> errors = generatorDiagnostics
            .Concat(compilation.GetDiagnostics())
            .Where(d => d.Severity == DiagnosticSeverity.Error)
            .ToList();
        if (errors.Count == 0)
            return compilation;

        throw new InvalidOperationException(
            "The cDAC source generator did not produce an error-free analysis compilation:" +
            Environment.NewLine +
            string.Join(Environment.NewLine, errors.Take(20).Select(d => $"  {d.Id}: {d.GetMessage()}")));
    }

    // Reference assemblies for the compilation: the trusted-platform-assembly set of the runtime
    // the tool is executing on ($(NetCoreAppToolCurrent)). This is a superset of what the cDAC
    // source needs (System.*), so the semantic model resolves BCL types without pulling in the
    // separate Basic.Reference.Assemblies package.
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
