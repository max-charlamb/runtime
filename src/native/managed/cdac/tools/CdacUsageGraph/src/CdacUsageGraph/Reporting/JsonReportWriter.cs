// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Text.Json;
using CdacUsageGraph.Model;

namespace CdacUsageGraph.Reporting;

/// <summary>Emits <c>contract-usage.json</c>: the full machine-readable usage graph.</summary>
public sealed class JsonReportWriter : IReportWriter
{
    private static readonly JsonSerializerOptions s_jsonOptions = new() { WriteIndented = true };

    // Named model (rather than an anonymous type) so every local can be explicitly typed. Property
    // names are the serialized JSON keys; declaration order is the emitted key order. dataTypes is
    // derived from the fieldUsage keys and kept for convenience of consumers.
    private sealed record ContractUsageJson(
        string contract,
        string version,
        string impl,
        string[] dataTypes,
        string[] contractsUsed,
        Dictionary<string, Dictionary<string, string[]>> fieldUsage);

    public string Write(UsageGraph graph, string outputDirectory)
    {
        List<ContractUsageJson> jsonModel = graph.Registrations
            .GroupBy(r => (r.Contract, r.Version)).Select(g => g.First())
            .OrderBy(r => r.Contract).ThenBy(r => r.Version)
            .Select(r =>
            {
                ContractLabel label = new ContractLabel(r.Contract, r.Version);
                graph.ContractsUsed.TryGetValue(label, out IReadOnlyCollection<string>? cset);
                Dictionary<string, Dictionary<string, string[]>> fields = graph.FieldUsage
                    .Where(k => k.Key.Label == label)
                    .OrderBy(k => k.Key.DataType, StringComparer.Ordinal)
                    .ToDictionary(
                        k => k.Key.DataType,
                        k => k.Value
                            .OrderBy(x => x.Key, StringComparer.Ordinal)
                            .ToDictionary(
                                x => x.Key,
                                x => x.Value.Select(u => u.ToString()).OrderBy(s => s, StringComparer.Ordinal).ToArray()));
                return new ContractUsageJson(
                    r.Contract,
                    r.Version,
                    r.Impl,
                    ReportQueries.DataTypesUsed(graph, label).ToArray(),
                    (cset ?? Array.Empty<string>()).OrderBy(x => x, StringComparer.Ordinal).ToArray(),
                    fields);
            })
            .ToList();

        File.WriteAllText(
            Path.Combine(outputDirectory, "contract-usage.json"),
            JsonSerializer.Serialize(jsonModel, s_jsonOptions));
        return "contract-usage.json";
    }
}
