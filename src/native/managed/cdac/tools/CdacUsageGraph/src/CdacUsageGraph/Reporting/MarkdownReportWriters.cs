// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Text;
using CdacUsageGraph.Model;

namespace CdacUsageGraph.Reporting;

/// <summary>Shared helpers for the report writers.</summary>
internal static class ReportQueries
{
    /// <summary>The Data types a contract uses -- the distinct <see cref="UsageGraph.FieldUsage"/>
    /// keys for the label -- sorted ordinally.</summary>
    public static List<string> DataTypesUsed(UsageGraph graph, ContractLabel label) =>
        graph.FieldUsage.Keys
            .Where(k => k.Label == label)
            .Select(k => k.DataType)
            .OrderBy(x => x, StringComparer.Ordinal)
            .ToList();
}

/// <summary>Emits <c>contract-data-graph.md</c>: (contract, version) -&gt; Data types used.</summary>
internal sealed class DataGraphMarkdownWriter : IReportWriter
{
    public string Write(UsageGraph graph, string outputDirectory)
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("# cDAC Contract -> Data Type Usage Graph");
        sb.AppendLine();
        sb.AppendLine($"_Generated from `{graph.CdacRoot}`. {graph.Registrations.Count} registrations, {graph.DataTypeCount} Data types._");
        sb.AppendLine();
        sb.AppendLine("| Contract | Version | # Data types | Data types used |");
        sb.AppendLine("|---|---|--:|---|");
        foreach ((string contract, string version, string _) in graph.Registrations
                     .GroupBy(r => (r.Contract, r.Version)).Select(g => g.First())
                     .OrderBy(r => r.Contract).ThenBy(r => r.Version))
        {
            List<string> list = ReportQueries.DataTypesUsed(graph, new ContractLabel(contract, version));
            sb.AppendLine($"| {contract} | {version} | {list.Count} | {string.Join(", ", list)} |");
        }

        File.WriteAllText(Path.Combine(outputDirectory, "contract-data-graph.md"), sb.ToString());
        return $"contract-data-graph.md ({graph.FieldUsage.Count} contract/data edges)";
    }
}

/// <summary>Emits <c>contract-field-usage.md</c>: (contract, version, Data type, field) -&gt; usage kind.</summary>
internal sealed class FieldUsageMarkdownWriter : IReportWriter
{
    public string Write(UsageGraph graph, string outputDirectory)
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("# cDAC Contract/Version -> Data Field Usage");
        sb.AppendLine();
        sb.AppendLine("UsageKind: Read = value read, Write / ReadWrite = mutation, OffsetLookup = `nameof(...)` field-offset reference, Size (field) = the type's overall size.");
        sb.AppendLine();
        sb.AppendLine("| Contract | Version | Data type | Field | Usage |");
        sb.AppendLine("|---|---|---|---|---|");
        foreach (KeyValuePair<(ContractLabel Label, string DataType), IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>> kv in graph.FieldUsage
                     .OrderBy(k => k.Key.Label.Contract, StringComparer.Ordinal)
                     .ThenBy(k => k.Key.Label.Version, StringComparer.Ordinal)
                     .ThenBy(k => k.Key.DataType, StringComparer.Ordinal))
        {
            (ContractLabel label, string dataType) = kv.Key;
            foreach (KeyValuePair<string, IReadOnlyCollection<UsageKind>> f in kv.Value.OrderBy(x => x.Key, StringComparer.Ordinal))
            {
                string kinds = string.Join("+", f.Value.Select(k => k.ToString()).OrderBy(s => s, StringComparer.Ordinal));
                sb.AppendLine($"| {label.Contract} | {label.Version} | {dataType} | {f.Key} | {kinds} |");
            }
        }

        File.WriteAllText(Path.Combine(outputDirectory, "contract-field-usage.md"), sb.ToString());
        return $"contract-field-usage.md ({graph.FieldUsage.Sum(x => x.Value.Count)} contract/field rows)";
    }
}

/// <summary>Emits <c>contract-global-usage.md</c>: globals read per contract/version.</summary>
internal sealed class GlobalUsageMarkdownWriter : IReportWriter
{
    public string Write(UsageGraph graph, string outputDirectory)
    {
        StringBuilder sb = new();
        sb.AppendLine("# cDAC Contract/Version -> Global Usage");
        sb.AppendLine();
        sb.AppendLine("| Contract | Version | Global | Type | Access |");
        sb.AppendLine("|---|---|---|---|---|");
        foreach (KeyValuePair<
            (ContractLabel Label, string Global),
            GlobalUsageInfo> entry in graph.GlobalUsage
                .OrderBy(entry => entry.Key.Label.Contract, StringComparer.Ordinal)
                .ThenBy(entry => entry.Key.Label.Version, StringComparer.Ordinal)
                .ThenBy(entry => entry.Key.Global, StringComparer.Ordinal))
        {
            string types = string.Join(
                " / ",
                entry.Value.Types.OrderBy(type => type, StringComparer.Ordinal));
            string access = entry.Value.IsOptional ? "Optional" : "Required";
            sb.AppendLine(
                $"| {entry.Key.Label.Contract} | {entry.Key.Label.Version} | " +
                $"{entry.Key.Global} | {types} | {access} |");
        }

        File.WriteAllText(
            Path.Combine(outputDirectory, "contract-global-usage.md"),
            sb.ToString());
        return $"contract-global-usage.md ({graph.GlobalUsage.Count} contract/global edges)";
    }
}

/// <summary>Emits <c>contract-contracts-used.md</c>: (contract, version) -&gt; other contracts used.</summary>
internal sealed class ContractsUsedMarkdownWriter : IReportWriter
{
    public string Write(UsageGraph graph, string outputDirectory)
    {
        StringBuilder sb = new StringBuilder();
        sb.AppendLine("# cDAC Contract/Version -> Contracts Used");
        sb.AppendLine();
        sb.AppendLine("Other contracts accessed via `_target.Contracts.<X>` (a contract dependency, not a data descriptor).");
        sb.AppendLine();
        sb.AppendLine("| Contract | Version | Contracts used |");
        sb.AppendLine("|---|---|---|");
        foreach ((string contract, string version, string _) in graph.Registrations
                     .GroupBy(r => (r.Contract, r.Version)).Select(g => g.First())
                     .OrderBy(r => r.Contract).ThenBy(r => r.Version))
        {
            graph.ContractsUsed.TryGetValue(new ContractLabel(contract, version), out IReadOnlyCollection<string>? cset);
            List<string> list = (cset ?? Array.Empty<string>()).OrderBy(x => x, StringComparer.Ordinal).ToList();
            sb.AppendLine($"| {contract} | {version} | {string.Join(", ", list)} |");
        }

        File.WriteAllText(Path.Combine(outputDirectory, "contract-contracts-used.md"), sb.ToString());
        return $"contract-contracts-used.md ({graph.ContractsUsed.Sum(x => x.Value.Count)} contract/contract edges)";
    }
}

/// <summary>Emits <c>contract-methods-reachable.md</c>: reachable method contexts per contract.</summary>
internal sealed class ReachableMethodsMarkdownWriter : IReportWriter
{
    public string Write(UsageGraph graph, string outputDirectory)
    {
        StringBuilder sb = new();
        sb.AppendLine("# cDAC Contract/Version -> Reachable Methods");
        sb.AppendLine();
        sb.AppendLine("| Contract | Version | Method |");
        sb.AppendLine("|---|---|---|");
        foreach ((string contract, string version, string _) in graph.Registrations
            .GroupBy(registration => (registration.Contract, registration.Version))
            .Select(group => group.First())
            .OrderBy(registration => registration.Contract)
            .ThenBy(registration => registration.Version))
        {
            ContractLabel label = new(contract, version);
            graph.ReachableMethods.TryGetValue(
                label,
                out IReadOnlyCollection<string>? methods);
            foreach (string method in (methods ?? []).OrderBy(
                method => method,
                StringComparer.Ordinal))
            {
                sb.AppendLine($"| {contract} | {version} | `{method}` |");
            }
        }

        File.WriteAllText(
            Path.Combine(outputDirectory, "contract-methods-reachable.md"),
            sb.ToString());
        return $"contract-methods-reachable.md ({graph.ReachableMethods.Sum(entry => entry.Value.Count)} contract/method contexts)";
    }
}
