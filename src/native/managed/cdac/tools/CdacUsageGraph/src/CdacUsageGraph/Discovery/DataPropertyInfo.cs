// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Discovery;

internal enum DataPropertyKind
{
    DirectField,
    TypeSize,
    Computed,
    OnInitDerived,
    ConstructorDerived,
}

/// <summary>
/// Precomputed provenance for a Data property. Direct fields are recorded by native name; derived
/// properties expand to the members that parse/compute their actual descriptor dependencies.
/// </summary>
internal sealed record DataPropertyInfo(
    DataPropertyKind Kind,
    string NativeName,
    IReadOnlyList<ISymbol> ExpansionMembers);
