// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace CdacUsageGraph.Discovery;

/// <summary>
/// Phase B (part 1): lookup index over the discovered <see cref="DataTypeInfo"/> objects.
/// Detection requires the real <c>IData&lt;TSelf&gt;</c> contract; cDAC names and property
/// provenance are owned by each <see cref="DataTypeInfo"/>, not parallel index dictionaries.
/// </summary>
internal sealed class DataTypeIndex
{
    private readonly Dictionary<INamedTypeSymbol, DataTypeInfo> _typesBySymbol;
    private readonly Dictionary<string, DataTypeInfo> _typesByName;

    private DataTypeIndex(
        Dictionary<INamedTypeSymbol, DataTypeInfo> typesBySymbol,
        Dictionary<string, DataTypeInfo> typesByName)
    {
        _typesBySymbol = typesBySymbol;
        _typesByName = typesByName;
    }

    public int Count => _typesBySymbol.Count;

    public IEnumerable<DataTypeInfo> Types => _typesBySymbol.Values;

    public bool TryGetDataType(ITypeSymbol? symbol, out DataTypeInfo info)
    {
        if (symbol is INamedTypeSymbol named)
            return _typesBySymbol.TryGetValue(named.OriginalDefinition, out info!);

        info = null!;
        return false;
    }

    public bool IsDataType(ITypeSymbol? symbol) => TryGetDataType(symbol, out _);

    /// <summary>Resolves a cDAC layout name from <c>GetTypeInfo(...)</c>.</summary>
    public bool TryGetType(string name, out DataTypeInfo info) =>
        _typesByName.TryGetValue(name, out info!);

    /// <summary>The discovered Data types that implement <paramref name="interfaceType"/>.</summary>
    public IEnumerable<DataTypeInfo> DataTypesImplementing(INamedTypeSymbol interfaceType) =>
        _typesBySymbol.Values.Where(info =>
            info.Symbol.AllInterfaces.Any(i =>
                SymbolEqualityComparer.Default.Equals(
                    i.OriginalDefinition, interfaceType.OriginalDefinition)));

    /// <summary>Builds the index by scanning every class in the generated Contracts compilation.</summary>
    public static DataTypeIndex Build(CSharpCompilation compilation)
    {
        SymbolEqualityComparer comparer = SymbolEqualityComparer.Default;
        INamedTypeSymbol? iDataDefinition = compilation.GetTypeByMetadataName(CdacSymbols.IDataMetadataName);
        if (iDataDefinition is null)
            throw new InvalidOperationException($"Could not resolve {CdacSymbols.IDataMetadataName}.");

        Dictionary<INamedTypeSymbol, DataTypeInfo> typesBySymbol =
            new Dictionary<INamedTypeSymbol, DataTypeInfo>(comparer);
        Dictionary<string, DataTypeInfo> typesByName =
            new Dictionary<string, DataTypeInfo>(StringComparer.Ordinal);

        foreach (INamedTypeSymbol candidate in EnumerateAllTypes(compilation.Assembly.GlobalNamespace))
        {
            if (candidate.TypeKind != TypeKind.Class ||
                !compilation.IsAssignableTo(candidate, iDataDefinition.Construct(candidate)))
                continue;

            DataTypeInfo info = DataTypeInfo.Create(compilation, candidate, comparer);
            typesBySymbol.Add(candidate, info);
            foreach (string name in info.Names)
                typesByName[name] = info;
        }

        return new DataTypeIndex(typesBySymbol, typesByName);
    }

    private static IEnumerable<INamedTypeSymbol> EnumerateAllTypes(INamespaceSymbol ns)
    {
        foreach (INamedTypeSymbol type in ns.GetTypeMembers())
            foreach (INamedTypeSymbol nested in EnumerateTypeAndNested(type))
                yield return nested;
        foreach (INamespaceSymbol child in ns.GetNamespaceMembers())
            foreach (INamedTypeSymbol type in EnumerateAllTypes(child))
                yield return type;
    }

    private static IEnumerable<INamedTypeSymbol> EnumerateTypeAndNested(INamedTypeSymbol type)
    {
        yield return type;
        foreach (INamedTypeSymbol nested in type.GetTypeMembers())
            foreach (INamedTypeSymbol descendant in EnumerateTypeAndNested(nested))
                yield return descendant;
    }
}
