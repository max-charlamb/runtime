# Contract ConditionalWeakTable

This contract provides the ability to look up values in a `ConditionalWeakTable<TKey, TValue>` managed object by key identity.

## APIs of contract

``` csharp
// Try to find the value associated with the given key in the conditional weak table.
// Returns true and sets value if found, false otherwise.
bool TryGetValue(TargetPointer conditionalWeakTable, TargetPointer key, out TargetPointer value);
```

## Version 1

This contract reads the field layout of `ConditionalWeakTable<TKey, TValue>` and its nested types
(`Container`, `Container+Entry`) via the [`ManagedTypeLayout`](ManagedTypeLayout.md) contract
rather than via cDAC data descriptors. Field offsets are resolved by name at runtime.

Data descriptors used:
| Data Descriptor Name | Field | Meaning |
| --- | --- | --- |
| `Array` | `m_NumComponents` | Number of elements in the array |

Contracts used:
| Contract Name |
| --- |
| `Object` |
| `GC` |
| `ManagedTypeLayout` |
| `RuntimeTypeSystem` |

## Managed Types

Field offsets for the managed types below are resolved via the
[`ManagedTypeLayout`](ManagedTypeLayout.md) contract.

| Managed Type | Field | Meaning |
| --- | --- | --- |
| `System.Runtime.CompilerServices.ConditionalWeakTable\`2` | `_container` | Pointer to the active `Container` holding buckets and entries |
| `System.Runtime.CompilerServices.ConditionalWeakTable\`2+Container` | `_buckets` | `int[]` buckets array; each slot is an index into `_entries` or `-1` |
| `System.Runtime.CompilerServices.ConditionalWeakTable\`2+Container` | `_entries` | `Entry[]` storage for the table's entries |
| `System.Runtime.CompilerServices.ConditionalWeakTable\`2+Entry` | `HashCode` | Hash code of the key (masked to positive int); chain terminator when `-1` |
| `System.Runtime.CompilerServices.ConditionalWeakTable\`2+Entry` | `Next` | Index of the next entry in the bucket chain, or `-1` |
| `System.Runtime.CompilerServices.ConditionalWeakTable\`2+Entry` | `depHnd` | Dependent handle tying the key to the value |

The algorithm looks up the `_container` field of the `ConditionalWeakTable` object, then reads the
`_buckets` and `_entries` fields from the container. It resolves `Entry` field offsets (`HashCode`,
`Next`, `depHnd`) via the [`ManagedTypeLayout`](ManagedTypeLayout.md) contract and determines the
entry stride from the entries array's component size.

`ManagedTypeInfo.Layout.Fields[name].Offset` is pre-shifted by `sizeof(Object)`, so for
reference-type fields read from an object address the canonical `address + field.Offset` idiom
applies. `Entry` is a value type stored inline in the `Entry[]` element storage (which has no
object header), so `Entry` field offsets are un-shifted by `sizeof(Object)` before being added to
the entry's address.

``` csharp
bool TryGetValue(TargetPointer conditionalWeakTable, TargetPointer key, out TargetPointer value)
{
    value = TargetPointer.Null;

    // Resolve managed type layouts from CoreLib via the ManagedTypeLayout contract.
    IManagedTypeLayout ml = target.Contracts.ManagedTypeLayout;
    IRuntimeTypeSystem rts = target.Contracts.RuntimeTypeSystem;
    ManagedTypeInfo cwtType = ml.GetTypeInfo("System.Runtime.CompilerServices", "ConditionalWeakTable`2");
    ManagedTypeInfo containerType = ml.GetTypeInfo("System.Runtime.CompilerServices", "ConditionalWeakTable`2+Container");
    ManagedTypeInfo entryType = ml.GetTypeInfo("System.Runtime.CompilerServices", "ConditionalWeakTable`2+Entry");

    uint objectSize = target.GetTypeInfo(DataType.Object).Size!.Value;

    // Navigate from the ConditionalWeakTable object to its container
    //   (reference-type reads: use field.Offset directly on the object address)
    TargetPointer container = target.ReadPointer(
        conditionalWeakTable + cwtType.Layout.Fields["_container"].Offset);

    // Read the container's buckets and entries array pointers
    TargetPointer bucketsPtr = target.ReadPointer(
        container + containerType.Layout.Fields["_buckets"].Offset);
    TargetPointer entriesPtr = target.ReadPointer(
        container + containerType.Layout.Fields["_entries"].Offset);

    // Entry is a value type stored inline in the Entry[] (no object header);
    // un-shift the pre-shifted field offsets by sizeof(Object).
    uint hashCodeOffset = entryType.Layout.Fields["HashCode"].Offset - objectSize;
    uint nextOffset     = entryType.Layout.Fields["Next"].Offset     - objectSize;
    uint depHndOffset   = entryType.Layout.Fields["depHnd"].Offset   - objectSize;

    // Get the runtime default hash code for the key object (returns 0 if none assigned)
    int hashCode = target.Contracts.Object.TryGetHashCode(key);
    if (hashCode == 0)
        return false;

    hashCode &= int.MaxValue;

    // Read the buckets array length and find the bucket (bucketCount is a power of 2)
    uint bucketCount = target.Read<uint>(bucketsPtr + /* Array::m_NumComponents offset */);
    int bucket = hashCode & (int)(bucketCount - 1);
    int entriesIndex = target.Read<int>(bucketsPtr + /* Array header size */ + bucket * sizeof(int));

    // Get entry size from the entries array's component size
    TargetPointer entriesMT = target.Contracts.Object.GetMethodTableAddress(entriesPtr);
    uint entrySize = rts.GetComponentSize(rts.GetTypeHandle(entriesMT));

    // Walk the chain
    while (entriesIndex != -1)
    {
        TargetPointer entryAddr = entriesPtr + /* Array header size */ + (uint)entriesIndex * entrySize;
        int entryHashCode = target.Read<int>(entryAddr + hashCodeOffset);

        if (entryHashCode == hashCode)
        {
            // depHnd is an OBJECTHANDLE — a pointer to a pointer to the object
            TargetPointer depHnd = target.ReadPointer(entryAddr + depHndOffset);
            TargetPointer handleTarget = target.ReadPointer(depHnd);
            if (handleTarget == key)
            {
                value = target.Contracts.GC.GetHandleExtraInfo(depHnd);
                return true;
            }
        }

        entriesIndex = target.Read<int>(entryAddr + nextOffset);
    }

    return false;
}
```
