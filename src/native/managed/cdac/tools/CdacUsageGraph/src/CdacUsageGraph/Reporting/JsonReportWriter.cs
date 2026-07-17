// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Text.Json;
using CdacUsageGraph.Model;

namespace CdacUsageGraph.Reporting;

/// <summary>Emits <c>contract-usage.json</c>: the full machine-readable usage graph.</summary>
internal sealed class JsonReportWriter : IReportWriter
{
    private static readonly JsonSerializerOptions s_jsonOptions = new() { WriteIndented = true };

    // Named model (rather than an anonymous type) so every local can be explicitly typed. Property
    // names are the serialized JSON keys; declaration order is the emitted key order. dataTypes is
    // derived from the fieldUsage keys and kept for convenience of consumers.
    private sealed record ContractUsageJson(
        string contract,
        string version,
        string[] impls,
        string[] dataTypes,
        string[] contractsUsed,
        string[] reachableMethods,
        Dictionary<string, GlobalUsageJson> globalsUsed,
        Dictionary<string, Dictionary<string, string[]>> fieldTypes,
        Dictionary<string, Dictionary<string, string[]>> fieldUsage);

    private sealed record GlobalUsageJson(
        string[] types,
        bool optional);

    public string Write(UsageGraph graph, string outputDirectory)
    {
        List<ContractUsageJson> jsonModel = graph.Registrations
            .GroupBy(r => (r.Contract, r.Version))
            .OrderBy(g => g.Key.Contract).ThenBy(g => g.Key.Version)
            .Select(g =>
            {
                ContractLabel label = new ContractLabel(g.Key.Contract, g.Key.Version);
                graph.ContractsUsed.TryGetValue(label, out IReadOnlyCollection<string>? cset);
                graph.ReachableMethods.TryGetValue(
                    label,
                    out IReadOnlyCollection<string>? methods);
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
                Dictionary<string, GlobalUsageJson> globals = graph.GlobalUsage
                    .Where(entry => entry.Key.Label == label)
                    .OrderBy(entry => entry.Key.Global, StringComparer.Ordinal)
                    .ToDictionary(
                        entry => entry.Key.Global,
                        entry => new GlobalUsageJson(
                            entry.Value.Types.OrderBy(
                                type => type,
                                StringComparer.Ordinal).ToArray(),
                            entry.Value.IsOptional),
                        StringComparer.Ordinal);
                Dictionary<string, Dictionary<string, string[]>> types = fields
                    .ToDictionary(
                        dataType => dataType.Key,
                        dataType => dataType.Value.Keys.ToDictionary(
                            field => field,
                            field => graph.FieldTypes.TryGetValue(
                                (dataType.Key, field),
                                out IReadOnlyCollection<string>? fieldTypes)
                                    ? fieldTypes.OrderBy(
                                        type => type,
                                        StringComparer.Ordinal).ToArray()
                                    : []),
                        StringComparer.Ordinal);
                return new ContractUsageJson(
                    g.Key.Contract,
                    g.Key.Version,
                    g.Select(r => r.Impl).Distinct(StringComparer.Ordinal)
                        .OrderBy(x => x, StringComparer.Ordinal).ToArray(),
                    ReportQueries.DataTypesUsed(graph, label).ToArray(),
                    (cset ?? Array.Empty<string>()).OrderBy(x => x, StringComparer.Ordinal).ToArray(),
                    (methods ?? Array.Empty<string>()).OrderBy(
                        method => method,
                        StringComparer.Ordinal).ToArray(),
                    globals,
                    types,
                    fields);
            })
            .ToList();

        File.WriteAllText(
            Path.Combine(outputDirectory, "contract-usage.json"),
            JsonSerializer.Serialize(jsonModel, s_jsonOptions));
        return "contract-usage.json";
    }
}
