// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Collections.Generic;
using System.Reflection;
using System.Reflection.Metadata;
using System.Reflection.Metadata.Ecma335;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

internal sealed class ManagedTypeLayout_1 : IManagedTypeLayout
{
    private readonly Target _target;
    private readonly Dictionary<(string, string), ManagedTypeInfo> _typeInfoCache = new();

    public ManagedTypeLayout_1(Target target)
    {
        _target = target;
    }

    public void Flush()
    {
        _typeInfoCache.Clear();
    }

    ManagedTypeInfo IManagedTypeLayout.GetTypeInfo(string @namespace, string typeName)
    {
        if (!((IManagedTypeLayout)this).TryGetTypeInfo(@namespace, typeName, out ManagedTypeInfo info))
        {
            throw new KeyNotFoundException($"Managed type '{@namespace}.{typeName}' was not found in the system assembly's metadata.");
        }
        return info;
    }

    bool IManagedTypeLayout.TryGetTypeInfo(string @namespace, string typeName, out ManagedTypeInfo info)
    {
        (string, string) key = (@namespace, typeName);
        if (_typeInfoCache.TryGetValue(key, out info))
            return true;

        if (!TryBuildTypeInfo(@namespace, typeName, out info))
            return false;

        _typeInfoCache[key] = info;
        return true;
    }

    private bool TryBuildTypeInfo(string @namespace, string typeName, out ManagedTypeInfo info)
    {
        info = default;

        ILoader loader = _target.Contracts.Loader;
        TargetPointer systemAssembly = loader.GetSystemAssembly();
        ModuleHandle moduleHandle = loader.GetModuleHandleFromAssemblyPtr(systemAssembly);
        IRuntimeTypeSystem rts = _target.Contracts.RuntimeTypeSystem;
        TypeHandle th = rts.GetTypeByNameAndModule(typeName, @namespace, moduleHandle);
        if (th.Address == TargetPointer.Null)
            return false;

        MetadataReader mdReader = _target.Contracts.EcmaMetadata.GetMetadata(moduleHandle)!;
        uint typeDefToken = rts.GetTypeDefToken(th);
        TypeDefinitionHandle typeDefHandle = (TypeDefinitionHandle)MetadataTokens.Handle((int)typeDefToken);
        TypeDefinition typeDef = mdReader.GetTypeDefinition(typeDefHandle);

        uint objectSize = _target.GetTypeInfo(DataType.Object).Size!.Value;

        Dictionary<string, Target.FieldInfo> instanceFields = new();
        Dictionary<string, TargetPointer> staticFields = new();

        foreach (FieldDefinitionHandle fieldHandle in typeDef.GetFields())
        {
            FieldDefinition fieldDef = mdReader.GetFieldDefinition(fieldHandle);
            string name = mdReader.GetString(fieldDef.Name);
            bool isStatic = (fieldDef.Attributes & FieldAttributes.Static) != 0;

            TargetPointer fieldDescAddr = rts.GetFieldDescByName(th, name);
            if (fieldDescAddr == TargetPointer.Null)
                continue;

            if (isStatic)
            {
                staticFields[name] = rts.GetFieldDescStaticAddress(fieldDescAddr);
            }
            else
            {
                uint fdOffset = rts.GetFieldDescOffset(fieldDescAddr, fieldDef);
                CorElementType elementType = rts.GetFieldDescType(fieldDescAddr);
                instanceFields[name] = new Target.FieldInfo
                {
                    Offset = (int)(fdOffset + objectSize),
                    TypeName = MapCorElementTypeToDescriptorName(elementType),
                };
            }
        }

        info = new ManagedTypeInfo
        {
            Layout = new Target.TypeInfo
            {
                Size = objectSize,
                Fields = instanceFields,
            },
            StaticFields = staticFields,
        };
        return true;
    }

    /// <summary>
    /// Maps an ECMA-335 <see cref="CorElementType"/> to the descriptor-type-name string used by
    /// <see cref="TargetFieldExtensions"/> debug assertions. Returns null when no precise mapping
    /// applies (the assertions treat null/empty TypeName as "skip validation").
    /// </summary>
    private static string? MapCorElementTypeToDescriptorName(CorElementType type) => type switch
    {
        CorElementType.Boolean => "bool",
        CorElementType.I1 => "int8",
        CorElementType.U1 => "uint8",
        CorElementType.Char or CorElementType.U2 => "uint16",
        CorElementType.I2 => "int16",
        CorElementType.I4 => "int32",
        CorElementType.U4 => "uint32",
        CorElementType.I8 => "int64",
        CorElementType.U8 => "uint64",
        // Object-reference and pointer-shaped element types: align with the native descriptor
        // convention that these were labeled "pointer" in datadescriptor.inc.
        CorElementType.String
            or CorElementType.Ptr
            or CorElementType.Byref
            or CorElementType.Class
            or CorElementType.Array
            or CorElementType.SzArray
            or CorElementType.GenericInst
            or CorElementType.Object
            or CorElementType.Var
            or CorElementType.MVar
            or CorElementType.FnPtr
            or CorElementType.I
            or CorElementType.U => "pointer",
        _ => null,
    };
}
