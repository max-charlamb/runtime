// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader.Data;

/// <summary>
/// Element-relative field offsets for <c>ConditionalWeakTable`2+Entry</c>.
/// </summary>
/// <remarks>
/// Entries are value types laid out inline in the <c>Entry[]</c> element storage (no object
/// header), so the object-start-relative offsets produced by <see cref="IManagedTypeLayout"/>
/// are un-shifted by <c>sizeof(Object)</c> to become element-relative. Not an <see cref="IData{T}"/>
/// because an Entry has no single managed-object address of its own.
/// </remarks>
internal sealed class ConditionalWeakTableEntry
{
    public const string Namespace = ConditionalWeakTable.Namespace;
    public const string Name = "ConditionalWeakTable`2+Entry";

    public ConditionalWeakTableEntry(Target target)
    {
        ManagedTypeInfo type = target.Contracts.ManagedTypeLayout.GetTypeInfo(Namespace, Name);
        int objectSize = (int)target.GetTypeInfo(DataType.Object).Size!.Value;

        HashCodeOffset = type.Layout.Fields["HashCode"].Offset - objectSize;
        NextOffset = type.Layout.Fields["Next"].Offset - objectSize;
        DepHndOffset = type.Layout.Fields["depHnd"].Offset - objectSize;
    }

    public int HashCodeOffset { get; }
    public int NextOffset { get; }
    public int DepHndOffset { get; }
}
