# Contract ManagedTypeLayout

This contract centralizes the lookup of managed-type (CoreLib) layout information —
instance field offsets and static field addresses — that was previously duplicated
across individual consumers (e.g. `ConditionalWeakTable`, `ComWrappers`, `SyncBlock`)
and across data readers for `Exception`, `String`, `NativeObjectWrapperObject`, and
`ManagedObjectWrapperHolderObject`.

Rather than hard-coding these managed-type field layouts as native
`CDAC_TYPE_*` entries in `datadescriptor.inc`, this contract resolves them at runtime
by looking up the corresponding managed metadata (namespace / type / field name).
This avoids drift between native descriptors and the managed source of truth, and
lets a single shared cache serve all consumers.

## APIs of contract

``` csharp
// Resolve the full layout (instance fields + static field slot addresses) for a managed type.
// Throws if the type cannot be found.
ManagedTypeInfo GetTypeInfo(string @namespace, string typeName);
bool TryGetTypeInfo(string @namespace, string typeName, out ManagedTypeInfo info);
```

The returned `ManagedTypeInfo` carries both instance-field layout (compatible with
the standard `Target.TypeInfo`-based field helpers such as `ReadPointerField` /
`ReadField<T>`) and static-field storage-slot addresses:

``` csharp
public readonly record struct ManagedTypeInfo
{
    // The runtime TypeHandle for this managed type (useful for identity comparisons,
    // e.g. comparing an object's method table against a known managed type).
    public TypeHandle TypeHandle { get; init; }

    // Instance layout. Size is sizeof(Object) (the size of the object header), and each
    // Fields[name].Offset is pre-shifted by sizeof(Object) so that callers can read a field
    // from an object via `address + field.Offset`, matching the idiom used by every other
    // Data.* type (e.g. Data.Object, Data.Exception).
    public Target.TypeInfo Layout { get; init; }

    // Absolute storage-slot addresses for static fields, keyed by field name.
    // Callers read the value/pointer from the returned address.
    public IReadOnlyDictionary<string, TargetPointer> StaticFields { get; init; }

    // Transparent conversion so the existing TargetFieldExtensions helpers
    // (ReadPointerField, ReadField<T>, ReadDataField<T>, …) accept ManagedTypeInfo directly.
    public static implicit operator Target.TypeInfo(ManagedTypeInfo info) => info.Layout;
}
```

### Usage patterns

Instance fields (the idiomatic shape — mirrors every other `Data.*` class):

``` csharp
ManagedTypeInfo type = target.Contracts.ManagedTypeLayout.GetTypeInfo("System", "Exception");
TargetPointer message = target.ReadPointerField(address, type, "_message");
int hResult = target.ReadField<int>(address, type, "_HResult");
```

Static fields:

``` csharp
ManagedTypeInfo cwType = ml.GetTypeInfo("System.Runtime.InteropServices", "ComWrappers");
TargetPointer slot = cwType.StaticFields["s_allManagedObjectWrapperTable"];
TargetPointer tableRef = target.ReadPointer(slot);
```

Value-type fields embedded inline (e.g. `ConditionalWeakTable`+`Entry` stored in an
array's element storage): because `Fields[name].Offset` is pre-shifted by
`sizeof(Object)` for use on object addresses, callers reading value-type storage
that does *not* include an object header must subtract `sizeof(Object)` back out to
obtain the element-relative offset.

## Version 1

All lookups are against the system assembly (CoreLib). The implementation delegates
to `IRuntimeTypeSystem` primitives (`GetTypeByNameAndModule`, `GetTypeDefToken`,
`GetFieldDescByName`, `GetFieldDescOffset`, `GetFieldDescStaticAddress`,
`GetFieldDescType`) plus the module's ECMA `MetadataReader` to enumerate
`TypeDefinition.GetFields()`, and caches the resulting `ManagedTypeInfo` per
`(namespace, typeName)` for the lifetime of the contract registry.

Data descriptors used: none — the entire point of the contract is that managed-type
layouts are not described by native data descriptors.

Contracts used:
| Contract Name |
| --- |
| `Loader` |
| `RuntimeTypeSystem` |
| `EcmaMetadata` |

``` csharp
ManagedTypeInfo GetTypeInfo(string @namespace, string typeName)
{
    if (_typeInfoCache.TryGetValue((@namespace, typeName), out ManagedTypeInfo cached))
        return cached;

    IRuntimeTypeSystem rts = target.Contracts.RuntimeTypeSystem;
    TargetPointer systemAssembly = target.Contracts.Loader.GetSystemAssembly();
    ModuleHandle moduleHandle = target.Contracts.Loader.GetModuleHandleFromAssemblyPtr(systemAssembly);
    TypeHandle th = rts.GetTypeByNameAndModule(typeName, @namespace, moduleHandle);
    MetadataReader reader = target.Contracts.EcmaMetadata.GetMetadata(moduleHandle);

    uint typeDefToken = rts.GetTypeDefToken(th);
    TypeDefinition typeDef = reader.GetTypeDefinition(
        (TypeDefinitionHandle)MetadataTokens.Handle((int)typeDefToken));

    uint objectSize = target.GetTypeInfo(DataType.Object).Size!.Value;
    var instanceFields = new Dictionary<string, Target.FieldInfo>();
    var staticFields = new Dictionary<string, TargetPointer>();

    foreach (FieldDefinitionHandle fdh in typeDef.GetFields())
    {
        FieldDefinition fieldDef = reader.GetFieldDefinition(fdh);
        string name = reader.GetString(fieldDef.Name);
        TargetPointer fd = rts.GetFieldDescByName(th, name);
        if (fd == TargetPointer.Null) continue;

        if ((fieldDef.Attributes & FieldAttributes.Static) != 0)
        {
            staticFields[name] = rts.GetFieldDescStaticAddress(fd);
        }
        else
        {
            uint rawOffset = rts.GetFieldDescOffset(fd, fieldDef);
            instanceFields[name] = new Target.FieldInfo
            {
                Offset = (int)(rawOffset + objectSize),
                TypeName = MapCorElementTypeToDescriptorName(rts.GetFieldDescType(fd)),
            };
        }
    }

    ManagedTypeInfo info = new ManagedTypeInfo
    {
        TypeHandle = th,
        Layout = new Target.TypeInfo { Size = objectSize, Fields = instanceFields },
        StaticFields = staticFields,
    };
    _typeInfoCache[(@namespace, typeName)] = info;
    return info;
}
```

### Notes

- **Pre-shift convention.** `Layout.Size = sizeof(Object)` and each
  `Fields[name].Offset = GetFieldDescOffset(fd) + sizeof(Object)`. Callers use
  `address + field.Offset` (or the `TargetFieldExtensions` helpers `ReadPointerField`
  / `ReadField<T>`) exactly as they would with any native `DataType.*`-backed type.
  This keeps the shape of managed-type consumption identical to native-type
  consumption.
- **Field `TypeName` mapping.** `CorElementType` values are mapped to the descriptor
  primitive strings (`"int32"`, `"uint16"`, …) or to `"pointer"` for reference-type
  shapes (`ELEMENT_TYPE_STRING`, `CLASS`, `SZARRAY`, `OBJECT`, `I`, `U`, …). This
  preserves the debug-mode primitive-type assertions in `TargetFieldExtensions`.
- **Static fields.** `info.StaticFields[name]` is the absolute storage-slot address
  returned by `GetFieldDescStaticAddress`. Callers still perform their own
  `ReadPointer` / `Read<T>` off that address.
- **TypeHandle.** `info.TypeHandle` is the runtime `TypeHandle` for the resolved
  type. Useful for identity comparisons — e.g. comparing an object's method table
  (`Object.GetMethodTableAddress(obj)`) against `info.TypeHandle.Address` without
  going back through `IRuntimeTypeSystem.GetTypeByNameAndModule`.
- **Value types.** The pre-shift convention is targeted at reference types read off
  object addresses. For value-type fields laid out inline (e.g. elements of a
  `T[]` where `T` is a struct), callers must un-shift by `sizeof(Object)` to obtain
  the element-relative offset.
- **Scope.** The contract currently looks up only CoreLib-defined types. If a future
  consumer needs to read managed types from a non-system assembly, this contract may
  be extended with a `GetTypeInfoInModule(ModuleHandle, …)` overload without
  affecting the current API.
