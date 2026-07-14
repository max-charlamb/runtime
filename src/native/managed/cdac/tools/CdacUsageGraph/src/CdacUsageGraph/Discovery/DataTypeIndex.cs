// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Discovery;

/// <summary>
/// Phase B (part 1): the immutable index of Data types discovered in the compilation:
/// which types are <c>[CdacType]</c>/<c>IData&lt;T&gt;</c>, each descriptor field's native name,
/// and the map from descriptor name to Data class (for <c>Fields["..."]</c> correlation).
/// </summary>
public sealed class DataTypeIndex
{
    private const string CdacTypeAttr = "Microsoft.Diagnostics.DataContractReader.CdacTypeAttribute";
    private const string FieldAttr = "Microsoft.Diagnostics.DataContractReader.FieldAttribute";
    private const string FieldAddressAttr = "Microsoft.Diagnostics.DataContractReader.FieldAddressAttribute";

    private readonly HashSet<INamedTypeSymbol> _dataTypes;
    private readonly Dictionary<string, INamedTypeSymbol> _cdacNameToType;
    private readonly Dictionary<IPropertySymbol, string> _propertyNativeName;

    private DataTypeIndex(
        HashSet<INamedTypeSymbol> dataTypes,
        Dictionary<string, INamedTypeSymbol> cdacNameToType,
        Dictionary<IPropertySymbol, string> propertyNativeName)
    {
        _dataTypes = dataTypes;
        _cdacNameToType = cdacNameToType;
        _propertyNativeName = propertyNativeName;
    }

    public int Count => _dataTypes.Count;

    /// <summary>True if <paramref name="t"/> is one of the discovered Data types.</summary>
    public bool IsDataType(ITypeSymbol? t) =>
        t is INamedTypeSymbol n && _dataTypes.Contains(n.OriginalDefinition);

    /// <summary>True if <paramref name="property"/> is a <c>[Field]</c>/<c>[FieldAddress]</c> descriptor property.</summary>
    public bool IsField(IPropertySymbol property) =>
        _propertyNativeName.ContainsKey((IPropertySymbol)property.OriginalDefinition);

    /// <summary>
    /// True if a Data type's <c>OnInit</c> declares via <see cref="System.Diagnostics.CodeAnalysis.MemberNotNullAttribute"/>
    /// that it initializes <paramref name="property"/>. Such properties are parsed/derived by
    /// <c>OnInit</c> rather than being descriptor fields themselves; walking <c>OnInit</c> exposes
    /// the actual fields on which they depend.
    /// </summary>
    public static bool IsInitializedByOnInit(IPropertySymbol property)
    {
        foreach (IMethodSymbol method in property.ContainingType.GetMembers("OnInit").OfType<IMethodSymbol>())
        {
            foreach (AttributeData attribute in method.GetAttributes())
            {
                if (attribute.AttributeClass?.ToDisplayString() !=
                    "System.Diagnostics.CodeAnalysis.MemberNotNullAttribute")
                    continue;

                foreach (TypedConstant argument in attribute.ConstructorArguments)
                {
                    if (argument.Kind == TypedConstantKind.Array)
                    {
                        if (argument.Values.Any(v => v.Value is string name && name == property.Name))
                            return true;
                    }
                    else if (argument.Value is string name && name == property.Name)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /// <summary>The discovered Data types that implement <paramref name="interfaceType"/>.</summary>
    public IEnumerable<INamedTypeSymbol> DataTypesImplementing(INamedTypeSymbol interfaceType)
    {
        INamedTypeSymbol iface = interfaceType.OriginalDefinition;
        foreach (INamedTypeSymbol dt in _dataTypes)
            if (dt.AllInterfaces.Any(i => SymbolEqualityComparer.Default.Equals(i.OriginalDefinition, iface)))
                yield return dt;
    }

    /// <summary>Resolves a descriptor name (from <c>GetTypeInfo(DataType.X)</c>) to a Data class.</summary>
    public bool TryGetType(string cdacName, out INamedTypeSymbol type) =>
        _cdacNameToType.TryGetValue(cdacName, out type!);

    /// <summary>The native descriptor name for a <c>[Field]</c> property, else the property name.</summary>
    public string NativeName(IPropertySymbol property) =>
        _propertyNativeName.TryGetValue((IPropertySymbol)property.OriginalDefinition, out string? n)
            ? n
            : property.Name;

    /// <summary>Builds the index by scanning every type in the compilation's assembly.</summary>
    public static DataTypeIndex Build(Microsoft.CodeAnalysis.Compilation compilation)
    {
        SymbolEqualityComparer comparer = SymbolEqualityComparer.Default;
        HashSet<INamedTypeSymbol> dataTypes = new HashSet<INamedTypeSymbol>(comparer);
        Dictionary<string, INamedTypeSymbol> cdacNameToType = new Dictionary<string, INamedTypeSymbol>(StringComparer.Ordinal);
        Dictionary<IPropertySymbol, string> propertyNativeName = new Dictionary<IPropertySymbol, string>(comparer);

        void VisitType(INamedTypeSymbol t)
        {
            bool isData =
                t.GetAttributes().Any(a => a.AttributeClass?.ToDisplayString() == CdacTypeAttr) ||
                t.AllInterfaces.Any(i => i.Name == "IData");
            if (isData)
            {
                dataTypes.Add(t);
                foreach (IPropertySymbol m in t.GetMembers().OfType<IPropertySymbol>())
                {
                    if (m.GetAttributes().Any(a =>
                            a.AttributeClass?.ToDisplayString() is FieldAttr or FieldAddressAttr))
                        propertyNativeName[m] = FieldNativeName(m);
                }

                cdacNameToType.TryAdd(t.Name, t);
                AttributeData? cd = t.GetAttributes().FirstOrDefault(a => a.AttributeClass?.ToDisplayString() == CdacTypeAttr);
                if (cd is { ConstructorArguments.Length: > 0 })
                {
                    foreach (TypedConstant v in cd.ConstructorArguments[0].Values)
                        if (v.Value is string s)
                            cdacNameToType[s] = t;
                }
            }

            foreach (INamedTypeSymbol nested in t.GetTypeMembers())
                VisitType(nested);
        }

        void VisitNamespace(INamespaceSymbol ns)
        {
            foreach (INamedTypeSymbol t in ns.GetTypeMembers())
                VisitType(t);
            foreach (INamespaceSymbol child in ns.GetNamespaceMembers())
                VisitNamespace(child);
        }

        VisitNamespace(compilation.Assembly.GlobalNamespace);
        return new DataTypeIndex(dataTypes, cdacNameToType, propertyNativeName);
    }

    // Highest-priority explicit candidate name from [Field]/[FieldAddress], else the property name.
    private static string FieldNativeName(IPropertySymbol p)
    {
        AttributeData? attr = p.GetAttributes().FirstOrDefault(a =>
            a.AttributeClass?.ToDisplayString() is FieldAttr or FieldAddressAttr);
        if (attr is not null)
        {
            if (attr.ConstructorArguments.Length == 1
                && attr.ConstructorArguments[0].Kind == TypedConstantKind.Array
                && attr.ConstructorArguments[0].Values is { Length: > 0 } cargs
                && cargs[0].Value is string cn)
                return cn;
            foreach (KeyValuePair<string, TypedConstant> na in attr.NamedArguments)
                if (na.Key == "Names" && na.Value.Kind == TypedConstantKind.Array
                    && na.Value.Values is { Length: > 0 } vargs && vargs[0].Value is string nn)
                    return nn;
        }
        return p.Name;
    }
}
