// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Text.RegularExpressions;
using CdacUsageGraph.Model;

namespace CdacUsageGraph.Docs;

/// <summary>
/// Fills the <c>&lt;!-- BEGIN GENERATED: ... --&gt;</c> marker blocks in
/// <c>docs/design/datacontracts/*.md</c> from a <see cref="UsageGraph"/> merged with
/// <see cref="DocDescriptorMeanings"/>. This is the single source of truth for the generated
/// tables; both the <c>docs</c> command and the doc-drift unit test use it, and
/// <c>generate-docs.ps1</c> is a thin wrapper around the command.
/// </summary>
internal sealed partial class DocGenerator
{
    private readonly UsageGraph _graph;
    private readonly DocDescriptorMeanings _meanings;

    public DocGenerator(UsageGraph graph, DocDescriptorMeanings meanings)
    {
        _graph = graph;
        _meanings = meanings;
    }

    /// <summary>Rewrites every marker block in <paramref name="docsDir"/>; returns the changed file names.</summary>
    public IReadOnlyList<string> Emit(string docsDir)
    {
        List<string> changed = new();
        foreach (FileInfo md in EnumerateMarkedDocs(docsDir))
        {
            string text = File.ReadAllText(md.FullName);
            string rewritten = Rewrite(text);
            if (!string.Equals(rewritten, text, StringComparison.Ordinal))
            {
                File.WriteAllText(md.FullName, rewritten);
                changed.Add(md.Name);
            }
        }
        return changed;
    }

    /// <summary>Returns the names of docs whose marker blocks are out of date (empty = up to date).</summary>
    public IReadOnlyList<string> Check(string docsDir)
    {
        List<string> drifted = new();
        foreach (FileInfo md in EnumerateMarkedDocs(docsDir))
        {
            string text = File.ReadAllText(md.FullName);
            if (!string.Equals(Rewrite(text), text, StringComparison.Ordinal))
                drifted.Add(md.Name);
        }
        return drifted;
    }

    private static IEnumerable<FileInfo> EnumerateMarkedDocs(string docsDir)
    {
        foreach (FileInfo md in new DirectoryInfo(docsDir).EnumerateFiles("*.md").OrderBy(f => f.Name, StringComparer.Ordinal))
        {
            string text = File.ReadAllText(md.FullName);
            if (text.Contains("BEGIN GENERATED:", StringComparison.Ordinal) ||
                text.Contains("END GENERATED:", StringComparison.Ordinal))
                yield return md;
        }
    }

    private string Rewrite(string text)
    {
        ValidateMarkers(text);
        string newline = text.Contains("\r\n", StringComparison.Ordinal) ? "\r\n" : "\n";
        return MarkerRegex().Replace(text, m =>
        {
            string kind = m.Groups["kind"].Value;
            string contract = m.Groups["c"].Value;
            string version = m.Groups["v"].Value;
            string begin = $"<!-- BEGIN GENERATED: {kind} contract={contract} version={version} -->";
            string end = $"<!-- END GENERATED: {kind} contract={contract} version={version} -->";
            IReadOnlyList<string> table = BuildUsage(contract, version);
            return string.Join(newline, new[] { begin }.Concat(table).Append(end));
        });
    }

    private static void ValidateMarkers(string text)
    {
        MatchCollection begins = BeginMarkerRegex().Matches(text);
        MatchCollection ends = EndMarkerRegex().Matches(text);
        MatchCollection blocks = MarkerRegex().Matches(text);
        if (begins.Count != blocks.Count || ends.Count != blocks.Count)
        {
            throw new InvalidOperationException(
                $"Malformed generated-doc markers: found {begins.Count} BEGIN marker(s), " +
                $"{ends.Count} END marker(s), and {blocks.Count} complete block(s).");
        }

        HashSet<(string Kind, string Contract, string Version)> seen = new();
        foreach (Match block in blocks)
        {
            string kind = block.Groups["kind"].Value;
            if (kind != "usage")
                throw new InvalidOperationException($"Unknown generated-doc marker kind '{kind}'.");

            (string, string, string) key = (
                kind,
                block.Groups["c"].Value,
                block.Groups["v"].Value);
            if (!seen.Add(key))
            {
                throw new InvalidOperationException(
                    $"Duplicate generated-doc block: kind={key.Item1}, contract={key.Item2}, version={key.Item3}.");
            }
        }
    }

    private List<string> BuildDataDescriptors(string contractShort, string version)
    {
        ContractLabel label = new($"I{contractShort}", version);

        // "Type.Field" set (type prefix stripped of the leading "Data.").
        HashSet<string> keys = new(StringComparer.Ordinal);
        foreach (KeyValuePair<(ContractLabel Label, string DataType), IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>> kv in _graph.FieldUsage)
        {
            if (kv.Key.Label != label)
                continue;
            string type = StripDataPrefix(kv.Key.DataType);
            foreach (string field in kv.Value.Keys)
                keys.Add($"{type}.{field}");
        }
        foreach (string k in _meanings.Supplement(contractShort))
            keys.Add(k);
        foreach (string k in _meanings.Suppress(contractShort))
            keys.Remove(k);

        List<string> rows = new()
        {
            "| Data Descriptor | Field | Type | Meaning |",
            "| --- | --- | --- | --- |",
        };
        foreach (string key in keys
            .OrderBy(key => key.Substring(0, key.LastIndexOf('.')),
                StringComparer.OrdinalIgnoreCase)
            .ThenBy(key => key.Substring(key.LastIndexOf('.') + 1),
                StringComparer.OrdinalIgnoreCase))
        {
            int dot = key.LastIndexOf('.', StringComparison.Ordinal);
            string type = key.Substring(0, dot);
            string field = key.Substring(dot + 1);
            string dataType = "Data." + type;
            string nativeType = _graph.FieldTypes.TryGetValue(
                (dataType, field),
                out IReadOnlyCollection<string>? fieldTypes)
                    ? string.Join(
                        " / ",
                        fieldTypes.OrderBy(value => value, StringComparer.Ordinal))
                    : "unknown";
            rows.Add(
                $"| `{type}` | `{field}` | `{nativeType}` | " +
                $"{_meanings.Meaning(contractShort, key)} |");
        }
        return rows;
    }

    private List<string> BuildUsage(string contractShort, string version)
    {
        List<string> rows =
        [
            "### Data descriptors used",
            "",
        ];
        rows.AddRange(BuildDataDescriptors(contractShort, version));
        rows.Add("");
        rows.Add("### Global variables used");
        rows.Add("");
        rows.AddRange(BuildGlobalsUsed(contractShort, version));
        rows.Add("");
        rows.Add("### Contracts used");
        rows.Add("");
        rows.AddRange(BuildContractsUsed(contractShort, version));
        return rows;
    }

    private List<string> BuildGlobalsUsed(string contractShort, string version)
    {
        ContractLabel label = new($"I{contractShort}", version);
        List<string> rows =
        [
            "| Global | Type | Meaning |",
            "| --- | --- | --- |",
        ];
        foreach (KeyValuePair<
            (ContractLabel Label, string Global),
            GlobalUsageInfo> entry in _graph.GlobalUsage
                .Where(entry => entry.Key.Label == label)
                .OrderBy(
                    entry => entry.Key.Global,
                    StringComparer.OrdinalIgnoreCase))
        {
            string types = string.Join(
                " / ",
                entry.Value.Types.OrderBy(type => type, StringComparer.Ordinal));
            string meaning = _meanings.GlobalMeaning(
                contractShort,
                entry.Key.Global);
            rows.Add(
                $"| `{entry.Key.Global}` | `{types}` | {meaning} |");
        }
        return rows;
    }

    private List<string> BuildContractsUsed(string contractShort, string version)
    {
        ContractLabel label = new($"I{contractShort}", version);
        List<string> rows = new()
        {
            "| Contract Name |",
            "| --- |",
        };
        if (_graph.ContractsUsed.TryGetValue(label, out IReadOnlyCollection<string>? used))
        {
            foreach (string x in used.OrderBy(s => s, StringComparer.OrdinalIgnoreCase))
                rows.Add($"| `{x}` |");
        }
        return rows;
    }

    private static string StripDataPrefix(string dataType) =>
        dataType.StartsWith("Data.", StringComparison.Ordinal) ? dataType.Substring("Data.".Length) : dataType;

    [GeneratedRegex(@"<!-- BEGIN GENERATED: (?<kind>[\w-]+) contract=(?<c>\w+) version=(?<v>\w+) -->.*?<!-- END GENERATED: \k<kind> contract=\k<c> version=\k<v> -->", RegexOptions.Singleline)]
    private static partial Regex MarkerRegex();

    [GeneratedRegex(@"<!-- BEGIN GENERATED: (?<kind>[\w-]+) contract=(?<c>\w+) version=(?<v>\w+) -->")]
    private static partial Regex BeginMarkerRegex();

    [GeneratedRegex(@"<!-- END GENERATED: (?<kind>[\w-]+) contract=(?<c>\w+) version=(?<v>\w+) -->")]
    private static partial Regex EndMarkerRegex();
}
