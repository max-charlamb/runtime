// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Collections.Generic;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

/// <summary>
/// Resolves layout information for managed types defined in CoreLib by looking them up in ECMA
/// metadata. See docs/design/datacontracts/ManagedTypeLayout.md.
/// </summary>
public interface IManagedTypeLayout : IContract
{
    static string IContract.Name { get; } = nameof(ManagedTypeLayout);

    ManagedTypeInfo GetTypeInfo(string @namespace, string typeName) => throw new NotImplementedException();

    bool TryGetTypeInfo(string @namespace, string typeName, out ManagedTypeInfo info) => throw new NotImplementedException();
}

public readonly struct ManagedTypeLayout : IManagedTypeLayout
{
    // Everything throws NotImplementedException
}

/// <summary>
/// Layout information for a managed type, synthesized from ECMA metadata.
/// </summary>
public readonly record struct ManagedTypeInfo
{
    /// <summary>
    /// Instance-field layout. Field offsets are pre-shifted by <c>sizeof(Object)</c> so callers
    /// can use <c>address + field.Offset</c> from the object address.
    /// </summary>
    public Target.TypeInfo Layout { get; init; }

    /// <summary>
    /// Absolute storage-slot addresses for static fields, keyed by field name.
    /// </summary>
    public IReadOnlyDictionary<string, TargetPointer> StaticFields { get; init; }

    public ManagedTypeInfo()
    {
        Layout = default;
        StaticFields = new Dictionary<string, TargetPointer>();
    }

    public static implicit operator Target.TypeInfo(ManagedTypeInfo info) => info.Layout;
}
