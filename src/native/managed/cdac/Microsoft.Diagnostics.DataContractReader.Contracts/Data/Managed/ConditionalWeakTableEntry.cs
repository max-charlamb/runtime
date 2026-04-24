// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader.Data;

/// <summary>
/// A <c>ConditionalWeakTable`2+Entry</c> value read from an inline element address in the
/// <c>Entry[]</c> storage.
/// </summary>
/// <remarks>
/// Entries are value types laid out inline in the <c>Entry[]</c> element storage (no object
/// header), so the object-start-relative offsets produced by <see cref="IManagedTypeLayout"/>
/// are un-shifted by <c>sizeof(Object)</c> before reading.
/// </remarks>
internal sealed class ConditionalWeakTableEntry : IData<ConditionalWeakTableEntry>
{
    public const string Namespace = ConditionalWeakTable.Namespace;
    public const string Name = "ConditionalWeakTable`2+Entry";

    static ConditionalWeakTableEntry IData<ConditionalWeakTableEntry>.Create(Target target, TargetPointer address)
        => new ConditionalWeakTableEntry(target, address);

    public ConditionalWeakTableEntry(Target target, TargetPointer address)
    {
        ManagedTypeInfo type = target.Contracts.ManagedTypeLayout.GetTypeInfo(Namespace, Name);
        int objectSize = (int)target.GetTypeInfo(DataType.Object).Size!.Value;

        HashCode = target.Read<int>(address + (ulong)(type.Layout.Fields["HashCode"].Offset - objectSize));
        Next = target.Read<int>(address + (ulong)(type.Layout.Fields["Next"].Offset - objectSize));
        DepHnd = target.ReadPointer(address + (ulong)(type.Layout.Fields["depHnd"].Offset - objectSize));
    }

    public int HashCode { get; init; }
    public int Next { get; init; }
    public TargetPointer DepHnd { get; init; }
}
