// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Model;

/// <summary>Registration reduced to plain strings for reporting (no Roslyn symbols).</summary>
internal sealed record RegistrationInfo(string Contract, string Version, string Impl);

/// <summary>
/// Immutable result of the analysis: for each (<see cref="ContractLabel"/>, Data type) the
/// per-field usage kinds, plus the other contracts each contract depends on. The set of Data types
/// a contract uses is derivable from the <see cref="FieldUsage"/> keys (a type used with no field
/// read appears with an empty field set).
/// </summary>
internal sealed record UsageGraph(
    string CdacRoot,
    int DataTypeCount,
    IReadOnlyList<RegistrationInfo> Registrations,
    IReadOnlyDictionary<(ContractLabel Label, string DataType), IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>> FieldUsage,
    IReadOnlyDictionary<ContractLabel, IReadOnlyCollection<string>> ContractsUsed);
