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

Contract constants:
| Constant | Value | Meaning |
| --- | --- | --- |
| `CWTNamespace` | `System.Runtime.CompilerServices` | Namespace of the `ConditionalWeakTable` type |
| `CWTTypeName` | ``ConditionalWeakTable`2`` | Name of the `ConditionalWeakTable<TKey, TValue>` type |
| `ContainerTypeName` | ``ConditionalWeakTable`2+Container`` | Name of the nested `Container` type |
| `EntryTypeName` | ``ConditionalWeakTable`2+Entry`` | Name of the nested `Entry` value type |
| `ContainerFieldName` | `_container` | Field on `ConditionalWeakTable` pointing to the active container |
| `BucketsFieldName` | `_buckets` | Field on `Container` pointing to the `int[]` buckets array |
| `EntriesFieldName` | `_entries` | Field on `Container` pointing to the `Entry[]` entries array |
| `HashCodeFieldName` | `HashCode` | Field on `Entry` storing the hash code (masked to positive int) |
| `NextFieldName` | `Next` | Field on `Entry` storing the next index in the chain, or -1 |
| `DepHndFieldName` | `depHnd` | Field on `Entry` storing the dependent handle |

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
    ManagedTypeInfo cwtType = ml.GetTypeInfo(CWTNamespace, CWTTypeName);
    ManagedTypeInfo containerType = ml.GetTypeInfo(CWTNamespace, ContainerTypeName);
    ManagedTypeInfo entryType = ml.GetTypeInfo(CWTNamespace, EntryTypeName);

    uint objectSize = target.GetTypeInfo(DataType.Object).Size!.Value;

    // Navigate from the ConditionalWeakTable object to its container
    //   (reference-type reads: use field.Offset directly on the object address)
    TargetPointer container = target.ReadPointer(
        conditionalWeakTable + cwtType.Layout.Fields[ContainerFieldName].Offset);

    // Read the container's buckets and entries array pointers
    TargetPointer bucketsPtr = target.ReadPointer(
        container + containerType.Layout.Fields[BucketsFieldName].Offset);
    TargetPointer entriesPtr = target.ReadPointer(
        container + containerType.Layout.Fields[EntriesFieldName].Offset);

    // Entry is a value type stored inline in the Entry[] (no object header);
    // un-shift the pre-shifted field offsets by sizeof(Object).
    uint hashCodeOffset = entryType.Layout.Fields[HashCodeFieldName].Offset - objectSize;
    uint nextOffset     = entryType.Layout.Fields[NextFieldName].Offset     - objectSize;
    uint depHndOffset   = entryType.Layout.Fields[DepHndFieldName].Offset   - objectSize;

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
