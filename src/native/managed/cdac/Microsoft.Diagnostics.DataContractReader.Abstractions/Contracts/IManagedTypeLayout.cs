// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Collections.Generic;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

/// <summary>
/// Resolves layout information for managed types defined in the runtime's system assembly
/// (CoreLib) by looking them up in ECMA metadata. Consumers obtain a <see cref="ManagedTypeInfo"/>
/// that carries both instance-field layout (compatible with <see cref="Target.TypeInfo"/>-based
/// <c>TargetFieldExtensions</c> helpers) and absolute storage-slot addresses for static fields.
/// </summary>
public interface IManagedTypeLayout : IContract
{
    static string IContract.Name { get; } = nameof(ManagedTypeLayout);

    /// <summary>
    /// Gets the layout information for a managed type defined in the system assembly (CoreLib).
    /// Throws if the type cannot be found.
    /// </summary>
    ManagedTypeInfo GetTypeInfo(string @namespace, string typeName) => throw new NotImplementedException();

    /// <summary>
    /// Attempts to get the layout information for a managed type defined in the system assembly (CoreLib).
    /// </summary>
    bool TryGetTypeInfo(string @namespace, string typeName, out ManagedTypeInfo info) => throw new NotImplementedException();
}

public readonly struct ManagedTypeLayout : IManagedTypeLayout
{
    // Everything throws NotImplementedException
}

/// <summary>
/// Layout information for a managed type, synthesized from ECMA metadata.
/// </summary>
/// <remarks>
/// The <see cref="Layout"/> member carries the instance-field layout with field offsets
/// already pre-shifted by <c>sizeof(Object)</c> and <see cref="Target.TypeInfo.Size"/> set to
/// <c>sizeof(Object)</c>, so callers can use the canonical <c>address + field.Offset</c> idiom
/// and the <c>TargetFieldExtensions</c> helpers (<c>ReadPointerField</c>, <c>ReadField&lt;T&gt;</c>,
/// etc.) directly on the object address.
///
/// The implicit conversion to <see cref="Target.TypeInfo"/> lets <see cref="ManagedTypeInfo"/>
/// be passed wherever a <see cref="Target.TypeInfo"/> is expected.
/// </remarks>
public readonly record struct ManagedTypeInfo
{
    /// <summary>
    /// Instance-field layout. <see cref="Target.TypeInfo.Size"/> is the object-data offset
    /// (<c>sizeof(Object)</c>). Each <see cref="Target.FieldInfo.Offset"/> is the field's
    /// object-start-relative offset (i.e., <c>GetFieldDescOffset</c> plus <c>sizeof(Object)</c>).
    /// </summary>
    public Target.TypeInfo Layout { get; init; }

    /// <summary>
    /// Absolute storage-slot addresses for the type's static fields, keyed by field name.
    /// Callers dereference these addresses themselves (e.g. <c>target.ReadPointer</c>).
    /// </summary>
    public IReadOnlyDictionary<string, TargetPointer> StaticFields { get; init; }

    public ManagedTypeInfo()
    {
        Layout = default;
        StaticFields = new Dictionary<string, TargetPointer>();
    }

    /// <summary>
    /// Implicit conversion to <see cref="Target.TypeInfo"/> so <see cref="ManagedTypeInfo"/>
    /// composes with <c>TargetFieldExtensions</c> methods that take a <see cref="Target.TypeInfo"/>.
    /// </summary>
    public static implicit operator Target.TypeInfo(ManagedTypeInfo info) => info.Layout;
}
