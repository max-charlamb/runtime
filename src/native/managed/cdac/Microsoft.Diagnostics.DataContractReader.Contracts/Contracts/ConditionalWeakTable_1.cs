// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

internal struct ConditionalWeakTable_1 : IConditionalWeakTable
{
    private const string CWTNamespace = "System.Runtime.CompilerServices";
    private const string CWTTypeName = "ConditionalWeakTable`2";
    private const string ContainerTypeName = "ConditionalWeakTable`2+Container";
    private const string EntryTypeName = "ConditionalWeakTable`2+Entry";

    private readonly Target _target;

    internal ConditionalWeakTable_1(Target target)
    {
        _target = target;
    }

    bool IConditionalWeakTable.TryGetValue(TargetPointer conditionalWeakTable, TargetPointer key, out TargetPointer value)
    {
        value = TargetPointer.Null;
        IManagedTypeLayout ml = _target.Contracts.ManagedTypeLayout;

        // Read _container field from the ConditionalWeakTable object
        ManagedTypeInfo cwtType = ml.GetTypeInfo(CWTNamespace, CWTTypeName);
        TargetPointer container = _target.ReadPointerField(conditionalWeakTable, cwtType, "_container");

        // Read _buckets and _entries fields from the Container object
        ManagedTypeInfo containerType = ml.GetTypeInfo(CWTNamespace, ContainerTypeName);
        TargetPointer bucketsPtr = _target.ReadPointerField(container, containerType, "_buckets");
        TargetPointer entriesPtr = _target.ReadPointerField(container, containerType, "_entries");

        int hashCode = _target.Contracts.Object.TryGetHashCode(key);
        if (hashCode == 0)
            return false;

        hashCode &= int.MaxValue;

        // Read the buckets array
        Data.Array bucketsArray = _target.ProcessedData.GetOrAdd<Data.Array>(bucketsPtr);
        uint bucketCount = bucketsArray.NumComponents;

        int bucket = hashCode & (int)(bucketCount - 1);
        int entriesIndex = _target.Read<int>(bucketsArray.DataPointer + (ulong)(bucket * sizeof(int)));

        // Resolve Entry field layout via ManagedTypeLayout
        ManagedTypeInfo entryType = ml.GetTypeInfo(CWTNamespace, EntryTypeName);

        // Entry instance-field offsets from ManagedTypeInfo are object-start-relative (pre-shifted
        // by sizeof(Object)). Entries live inline in the array's element storage, which does not
        // include the object header — subtract Object.Size to obtain element-relative offsets.
        uint objectSize = _target.GetTypeInfo(DataType.Object).Size!.Value;
        int hashCodeElementOffset = entryType.Layout.Fields["HashCode"].Offset - (int)objectSize;
        int nextElementOffset = entryType.Layout.Fields["Next"].Offset - (int)objectSize;
        int depHndElementOffset = entryType.Layout.Fields["depHnd"].Offset - (int)objectSize;

        // Get entry size from the entries array's component size
        Data.Array entriesArray = _target.ProcessedData.GetOrAdd<Data.Array>(entriesPtr);
        IRuntimeTypeSystem rts = _target.Contracts.RuntimeTypeSystem;
        TargetPointer entriesMT = _target.Contracts.Object.GetMethodTableAddress(entriesPtr);
        TypeHandle entriesTypeHandle = rts.GetTypeHandle(entriesMT);
        uint entrySize = rts.GetComponentSize(entriesTypeHandle);

        while (entriesIndex != -1)
        {
            TargetPointer entryAddress = entriesArray.DataPointer + (ulong)((uint)entriesIndex * entrySize);

            int entryHashCode = _target.Read<int>(entryAddress + (ulong)hashCodeElementOffset);
            if (entryHashCode == hashCode)
            {
                TargetPointer depHnd = _target.ReadPointer(entryAddress + (ulong)depHndElementOffset);
                Data.ObjectHandle handle = _target.ProcessedData.GetOrAdd<Data.ObjectHandle>(depHnd);
                if (handle.Object == key)
                {
                    TargetNUInt extraInfo = _target.Contracts.GC.GetHandleExtraInfo(depHnd);
                    value = new TargetPointer(extraInfo.Value);

                    return true;
                }
            }

            entriesIndex = _target.Read<int>(entryAddress + (ulong)nextElementOffset);
        }

        return false;
    }
}
