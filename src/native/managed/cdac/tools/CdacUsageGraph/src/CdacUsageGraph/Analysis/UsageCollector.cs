// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Model;

namespace CdacUsageGraph.Analysis;

/// <summary>
/// Mutable accumulator for the walk. <see cref="FieldUsage"/> is the single source of truth: it
/// records, per (<see cref="ContractLabel"/>, Data type), the per-field usage kinds. A Data type
/// used without any recorded field (e.g. materialized via <c>GetOrAdd&lt;Data.X&gt;</c> only)
/// still appears -- as an entry with an empty field set -- so the set of Data types used is simply
/// the keys. Frozen into the immutable <see cref="UsageGraph"/> when the walk completes.
/// </summary>
internal sealed class UsageCollector
{
    private readonly Dictionary<(ContractLabel, string), Dictionary<string, HashSet<UsageKind>>> _fieldUsage = new();
    private readonly Dictionary<ContractLabel, HashSet<string>> _contractsUsed = new();
    private readonly Dictionary<ContractLabel, HashSet<string>> _reachableMethods = new();

    /// <summary>Records that <paramref name="dataName"/> is used, even if no field is read.</summary>
    public void RecordType(ContractLabel label, string dataName) => GetOrAddType(label, dataName);

    /// <summary>Records a specific field usage (and implicitly the type usage).</summary>
    public void RecordField(ContractLabel label, string dataName, string field, UsageKind kind)
    {
        Dictionary<string, HashSet<UsageKind>> fields = GetOrAddType(label, dataName);
        if (!fields.TryGetValue(field, out HashSet<UsageKind>? kinds))
            fields[field] = kinds = new HashSet<UsageKind>();
        kinds.Add(kind);
    }

    public void RecordContractUsed(ContractLabel label, string contractName)
    {
        if (!_contractsUsed.TryGetValue(label, out HashSet<string>? set))
            _contractsUsed[label] = set = new HashSet<string>(StringComparer.Ordinal);
        set.Add(contractName);
    }

    public void RecordReachableMethod(ContractLabel label, string method)
    {
        if (!_reachableMethods.TryGetValue(label, out HashSet<string>? methods))
            _reachableMethods[label] = methods = new HashSet<string>(StringComparer.Ordinal);
        methods.Add(method);
    }

    public IReadOnlyDictionary<(ContractLabel Label, string DataType), IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>> FieldUsage =>
        _fieldUsage.ToDictionary(
            kv => kv.Key,
            kv => (IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>)
                kv.Value.ToDictionary(x => x.Key, x => (IReadOnlyCollection<UsageKind>)x.Value));

    public IReadOnlyDictionary<ContractLabel, IReadOnlyCollection<string>> ContractsUsed =>
        _contractsUsed.ToDictionary(kv => kv.Key, kv => (IReadOnlyCollection<string>)kv.Value);

    public IReadOnlyDictionary<ContractLabel, IReadOnlyCollection<string>> ReachableMethods =>
        _reachableMethods.ToDictionary(
            kv => kv.Key,
            kv => (IReadOnlyCollection<string>)kv.Value);

    private Dictionary<string, HashSet<UsageKind>> GetOrAddType(ContractLabel label, string dataName)
    {
        (ContractLabel, string) key = (label, dataName);
        if (!_fieldUsage.TryGetValue(key, out Dictionary<string, HashSet<UsageKind>>? fields))
            _fieldUsage[key] = fields = new Dictionary<string, HashSet<UsageKind>>(StringComparer.Ordinal);
        return fields;
    }
}
