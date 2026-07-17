// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Collections.Immutable;
using CdacUsageGraph.Compilation;
using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// Exact Roslyn symbols for the primitive cDAC TypeInfo APIs. Each API stores equivalent symbols
/// from every analyzable compilation, so calls resolved through a project reference and calls in
/// the defining project match the same semantic API.
/// </summary>
internal sealed class CdacApiSymbols
{
    private CdacApiSymbols(
        ApiMethod targetGetTypeInfo,
        ApiMethod targetTryGetTypeInfo,
        ApiMethod? dataTypeToName,
        ApiMethod managedGetTypeInfo,
        ApiMethod managedTryGetTypeInfo,
        ApiType typeInfoType,
        ApiProperty typeInfoFields,
        ApiProperty typeInfoSize,
        ApiProperty fieldInfoOffset,
        ApiMethod? dataCacheGetOrAdd,
        ApiMethod targetReadGlobal,
        ApiMethod targetTryReadGlobal,
        ApiMethod targetReadGlobalPointer,
        ApiMethod targetTryReadGlobalPointer,
        ApiMethod targetReadGlobalString,
        ApiMethod targetTryReadGlobalString,
        ApiMethod targetReadMethods,
        ApiMethod targetWriteMethods)
    {
        TargetGetTypeInfo = targetGetTypeInfo;
        TargetTryGetTypeInfo = targetTryGetTypeInfo;
        DataTypeToName = dataTypeToName;
        ManagedGetTypeInfo = managedGetTypeInfo;
        ManagedTryGetTypeInfo = managedTryGetTypeInfo;
        TypeInfoType = typeInfoType;
        TypeInfoFields = typeInfoFields;
        TypeInfoSize = typeInfoSize;
        FieldInfoOffset = fieldInfoOffset;
        DataCacheGetOrAdd = dataCacheGetOrAdd;
        TargetReadGlobal = targetReadGlobal;
        TargetTryReadGlobal = targetTryReadGlobal;
        TargetReadGlobalPointer = targetReadGlobalPointer;
        TargetTryReadGlobalPointer = targetTryReadGlobalPointer;
        TargetReadGlobalString = targetReadGlobalString;
        TargetTryReadGlobalString = targetTryReadGlobalString;
        TargetReadMethods = targetReadMethods;
        TargetWriteMethods = targetWriteMethods;
    }

    public ApiMethod TargetGetTypeInfo { get; }
    public ApiMethod TargetTryGetTypeInfo { get; }
    public ApiMethod ManagedGetTypeInfo { get; }
    public ApiMethod ManagedTryGetTypeInfo { get; }
    public ApiMethod? DataTypeToName { get; }
    public ApiType TypeInfoType { get; }
    public ApiProperty TypeInfoFields { get; }
    public ApiProperty TypeInfoSize { get; }
    public ApiProperty FieldInfoOffset { get; }
    public ApiMethod? DataCacheGetOrAdd { get; }
    public ApiMethod TargetReadGlobal { get; }
    public ApiMethod TargetTryReadGlobal { get; }
    public ApiMethod TargetReadGlobalPointer { get; }
    public ApiMethod TargetTryReadGlobalPointer { get; }
    public ApiMethod TargetReadGlobalString { get; }
    public ApiMethod TargetTryReadGlobalString { get; }
    public ApiMethod TargetReadMethods { get; }
    public ApiMethod TargetWriteMethods { get; }

    public static CdacApiSymbols Build(CdacAnalysisWorkspace workspace)
    {
        ImmutableArray<INamedTypeSymbol> targets =
            ResolveTypes(workspace, CdacSymbols.TargetMetadataName);
        ImmutableArray<INamedTypeSymbol> typeInfos =
            ResolveTypes(workspace, CdacSymbols.TargetTypeInfoMetadataName);
        ImmutableArray<INamedTypeSymbol> fieldInfos =
            ResolveTypes(workspace, CdacSymbols.TargetFieldInfoMetadataName);
        ImmutableArray<INamedTypeSymbol> dataCaches =
            ResolveTypes(workspace, CdacSymbols.TargetDataCacheMetadataName, required: false);
        ImmutableArray<INamedTypeSymbol> managedTypeSources =
            ResolveTypes(workspace, CdacSymbols.IManagedTypeSourceMetadataName);
        ImmutableArray<INamedTypeSymbol> dataTypeNames =
            ResolveTypes(workspace, CdacSymbols.DataTypeNamesMetadataName, required: false);

        return new CdacApiSymbols(
            CreateMethod(
                targets,
                CdacSymbols.GetTypeInfoMethodName,
                method => method.Parameters is
                [
                    { Type.SpecialType: SpecialType.System_String },
                ]),
            CreateMethod(
                targets,
                CdacSymbols.TryGetTypeInfoMethodName,
                method => method.ReturnType.SpecialType == SpecialType.System_Boolean &&
                    method.Parameters is
                    [
                        { Type.SpecialType: SpecialType.System_String },
                        { RefKind: RefKind.Out },
                    ]),
            dataTypeNames.Length == 0
                ? null
                : CreateMethod(
                    dataTypeNames,
                    CdacSymbols.ToNameMethodName,
                    method => method.Parameters is
                    [
                        { Type.TypeKind: TypeKind.Enum },
                    ]),
            CreateMethod(
                managedTypeSources,
                CdacSymbols.GetTypeInfoMethodName,
                method => method.Parameters is
                [
                    { Type.SpecialType: SpecialType.System_String },
                ]),
            CreateMethod(
                managedTypeSources,
                CdacSymbols.TryGetTypeInfoMethodName,
                method => method.ReturnType.SpecialType == SpecialType.System_Boolean &&
                    method.Parameters is
                    [
                        { Type.SpecialType: SpecialType.System_String },
                        { RefKind: RefKind.Out },
                    ]),
            new ApiType(typeInfos),
            CreateProperty(typeInfos, "Fields"),
            CreateProperty(typeInfos, "Size"),
            CreateProperty(fieldInfos, "Offset"),
            dataCaches.Length == 0
                ? null
                : CreateMethod(
                    dataCaches,
                    "GetOrAdd",
                    method => method.Parameters.Length == 1),
            CreateMethod(
                targets,
                CdacSymbols.ReadGlobalMethodName,
                method => method.TypeParameters.Length == 1 &&
                    method.Parameters is
                    [
                        { Type.SpecialType: SpecialType.System_String },
                    ]),
            CreateMethod(
                targets,
                CdacSymbols.TryReadGlobalMethodName,
                method => method.TypeParameters.Length == 1 &&
                    method.Parameters is
                    [
                        { Type.SpecialType: SpecialType.System_String },
                        { RefKind: RefKind.Out },
                    ]),
            CreateMethod(
                targets,
                CdacSymbols.ReadGlobalPointerMethodName,
                method => method.Parameters is
                [
                    { Type.SpecialType: SpecialType.System_String },
                ]),
            CreateMethod(
                targets,
                CdacSymbols.TryReadGlobalPointerMethodName,
                method => method.Parameters is
                [
                    { Type.SpecialType: SpecialType.System_String },
                    { RefKind: RefKind.Out },
                ]),
            CreateMethod(
                targets,
                CdacSymbols.ReadGlobalStringMethodName,
                method => method.Parameters is
                [
                    { Type.SpecialType: SpecialType.System_String },
                ]),
            CreateMethod(
                targets,
                CdacSymbols.TryReadGlobalStringMethodName,
                method => method.Parameters is
                [
                    { Type.SpecialType: SpecialType.System_String },
                    { RefKind: RefKind.Out },
                ]),
            CreateMethods(
                targets,
                [
                    "Read",
                    "TryRead",
                    "ReadPointer",
                    "ReadCodePointer",
                    "ReadNUInt",
                    "ReadNInt",
                    "ReadBuffer",
                ],
                HasAddressParameter),
            CreateMethods(
                targets,
                [
                    "Write",
                    "WritePointer",
                    "WriteNUInt",
                    "WriteBuffer",
                ],
                HasAddressParameter));
    }

    private static ImmutableArray<INamedTypeSymbol> ResolveTypes(
        CdacAnalysisWorkspace workspace,
        string metadataName,
        bool required = true)
    {
        ImmutableArray<INamedTypeSymbol>.Builder builder = ImmutableArray.CreateBuilder<INamedTypeSymbol>();
        foreach (Microsoft.CodeAnalysis.CSharp.CSharpCompilation compilation in workspace.AnalyzableCompilations)
        {
            if (compilation.GetTypeByMetadataName(metadataName) is INamedTypeSymbol type)
                builder.Add(type);
        }
        if (required && builder.Count == 0)
            throw new InvalidOperationException($"Could not resolve required cDAC type '{metadataName}'.");
        return builder.ToImmutable();
    }

    private static ApiMethod CreateMethod(
        ImmutableArray<INamedTypeSymbol> types,
        string name,
        Func<IMethodSymbol, bool> predicate)
    {
        ImmutableArray<IMethodSymbol>.Builder builder = ImmutableArray.CreateBuilder<IMethodSymbol>();
        foreach (INamedTypeSymbol type in types)
        {
            IMethodSymbol[] methods = type.GetMembers(name).OfType<IMethodSymbol>()
                .Where(predicate)
                .ToArray();
            if (methods.Length > 1)
                throw new InvalidOperationException($"Ambiguous cDAC API method '{type}.{name}'.");
            if (methods.Length == 1)
                builder.Add(methods[0].OriginalDefinition);
        }
        if (builder.Count == 0)
            throw new InvalidOperationException($"Could not resolve required cDAC API method '{name}'.");
        return new ApiMethod(builder.ToImmutable());
    }

    private static ApiMethod CreateMethods(
        ImmutableArray<INamedTypeSymbol> types,
        IReadOnlyCollection<string> names,
        Func<IMethodSymbol, bool> predicate)
    {
        ImmutableArray<IMethodSymbol>.Builder builder = ImmutableArray.CreateBuilder<IMethodSymbol>();
        foreach (INamedTypeSymbol type in types)
        {
            foreach (string name in names)
            {
                builder.AddRange(type.GetMembers(name).OfType<IMethodSymbol>()
                    .Where(predicate)
                    .Select(method => method.OriginalDefinition));
            }
        }
        if (builder.Count == 0)
            throw new InvalidOperationException(
                $"Could not resolve required cDAC API methods '{string.Join(", ", names)}'.");
        return new ApiMethod(builder.ToImmutable());
    }

    private static ApiProperty CreateProperty(
        ImmutableArray<INamedTypeSymbol> types,
        string name)
    {
        ImmutableArray<IPropertySymbol>.Builder builder = ImmutableArray.CreateBuilder<IPropertySymbol>();
        foreach (INamedTypeSymbol type in types)
        {
            IPropertySymbol[] properties = type.GetMembers(name).OfType<IPropertySymbol>().ToArray();
            if (properties.Length > 1)
                throw new InvalidOperationException($"Ambiguous cDAC API property '{type}.{name}'.");
            if (properties.Length == 1)
                builder.Add(properties[0].OriginalDefinition);
        }
        if (builder.Count == 0)
            throw new InvalidOperationException($"Could not resolve required cDAC API property '{name}'.");
        return new ApiProperty(builder.ToImmutable());
    }

    private static bool HasAddressParameter(IMethodSymbol method) =>
        method.Parameters.Length > 0 &&
        method.Parameters[0].Type.SpecialType == SpecialType.System_UInt64;
}

internal sealed class ApiMethod
{
    private readonly ImmutableArray<IMethodSymbol> _definitions;

    public ApiMethod(ImmutableArray<IMethodSymbol> definitions) => _definitions = definitions;

    public bool Matches(IMethodSymbol method)
    {
        IMethodSymbol candidate = method.ReducedFrom?.OriginalDefinition ??
            method.OriginalDefinition;
        return _definitions.Any(definition =>
            SymbolEqualityComparer.Default.Equals(definition, candidate));
    }
}

internal sealed class ApiProperty
{
    private readonly ImmutableArray<IPropertySymbol> _definitions;

    public ApiProperty(ImmutableArray<IPropertySymbol> definitions) => _definitions = definitions;

    public bool Matches(IPropertySymbol property) =>
        _definitions.Any(definition =>
            SymbolEqualityComparer.Default.Equals(definition, property.OriginalDefinition));
}

internal sealed class ApiType
{
    private readonly ImmutableArray<INamedTypeSymbol> _definitions;

    public ApiType(ImmutableArray<INamedTypeSymbol> definitions) => _definitions = definitions;

    public bool Matches(ITypeSymbol type) =>
        _definitions.Any(definition =>
            SymbolEqualityComparer.Default.Equals(
                definition.OriginalDefinition,
                type.OriginalDefinition));
}
