// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Discovery;

/// <summary>
/// Immutable description of one cDAC <c>IData&lt;TSelf&gt;</c> class: its Roslyn symbol, cDAC
/// names, and provenance-aware property model.
/// </summary>
internal sealed class DataTypeInfo
{
    private readonly Dictionary<IPropertySymbol, DataPropertyInfo> _properties;

    private DataTypeInfo(
        INamedTypeSymbol symbol,
        IReadOnlyList<string> names,
        Dictionary<IPropertySymbol, DataPropertyInfo> properties)
    {
        Symbol = symbol;
        Names = names;
        _properties = properties;
    }

    public INamedTypeSymbol Symbol { get; }

    /// <summary>
    /// The primary cDAC name, used in reports. This is the first name declared by
    /// <c>CdacType</c>, or the C# type name when no attribute supplies names.
    /// </summary>
    public string Name => Names[0];

    /// <summary>
    /// Ordered cDAC names usable for layout lookup. The C# type name is included as a fallback.
    /// No special handling of the <c>DataType</c> enum is required.
    /// </summary>
    public IReadOnlyList<string> Names { get; }

    public IEnumerable<DataPropertyInfo> Properties => _properties.Values;

    public DataPropertyInfo GetProperty(IPropertySymbol property) =>
        _properties[(IPropertySymbol)property.OriginalDefinition];

    public static DataTypeInfo Create(
        CSharpCompilation compilation,
        INamedTypeSymbol symbol,
        SymbolEqualityComparer comparer)
    {
        string[] names = GetNames(symbol);
        Dictionary<IPropertySymbol, DataPropertyInfo> properties =
            new Dictionary<IPropertySymbol, DataPropertyInfo>(comparer);

        foreach (IPropertySymbol property in EnumerateProperties(symbol))
            properties.TryAdd(property.OriginalDefinition, CreatePropertyInfo(compilation, property, comparer));

        return new DataTypeInfo(symbol, names, properties);
    }

    private static string[] GetNames(INamedTypeSymbol symbol)
    {
        AttributeData? attribute = symbol.GetAttributes().FirstOrDefault(
            a => a.AttributeClass?.ToDisplayString() == CdacSymbols.CdacTypeAttributeMetadataName);
        if (attribute is not { ConstructorArguments.Length: > 0 })
            return [symbol.Name];

        List<string> names = attribute.ConstructorArguments[0].Values
            .Where(value => value.Value is string)
            .Select(value => (string)value.Value!)
            .Distinct(StringComparer.Ordinal)
            .ToList();
        if (!names.Contains(symbol.Name, StringComparer.Ordinal))
            names.Add(symbol.Name);
        return names.ToArray();
    }

    private static IEnumerable<IPropertySymbol> EnumerateProperties(INamedTypeSymbol type)
    {
        for (INamedTypeSymbol? current = type; current is not null; current = current.BaseType)
        {
            foreach (IPropertySymbol property in current.GetMembers().OfType<IPropertySymbol>())
                yield return property;
        }
    }

    private static DataPropertyInfo CreatePropertyInfo(
        CSharpCompilation compilation,
        IPropertySymbol property,
        SymbolEqualityComparer comparer)
    {
        IPropertySymbol definition = property.OriginalDefinition;
        if (TryGetNativeFieldName(definition, out string? nativeName))
            return new DataPropertyInfo(definition, DataPropertyKind.DirectField, nativeName, []);

        if (definition.GetAttributes().Any(
            a => a.AttributeClass?.ToDisplayString() == CdacSymbols.InstanceDataStartAttributeMetadataName))
            return new DataPropertyInfo(definition, DataPropertyKind.TypeSize, "Size", []);

        if (HasComputedGetter(definition))
            return new DataPropertyInfo(definition, DataPropertyKind.Computed, property.Name, [definition]);

        List<ISymbol> onInitMembers = OnInitMembersInitializing(definition);
        if (onInitMembers.Count > 0)
            return new DataPropertyInfo(definition, DataPropertyKind.OnInitDerived, property.Name, onInitMembers);

        List<ISymbol> constructors = ConstructorsInitializing(compilation, definition, comparer);
        if (constructors.Count > 0)
            return new DataPropertyInfo(definition, DataPropertyKind.ConstructorDerived, property.Name, constructors);

        return new DataPropertyInfo(definition, DataPropertyKind.DirectField, property.Name, []);
    }

    private static bool HasComputedGetter(IPropertySymbol property)
    {
        foreach (SyntaxReference reference in property.DeclaringSyntaxReferences)
        {
            if (reference.GetSyntax() is not PropertyDeclarationSyntax declaration)
                continue;
            if (declaration.ExpressionBody is not null)
                return true;
            if (declaration.AccessorList is { } accessors &&
                accessors.Accessors.Any(a =>
                    a.Kind() == SyntaxKind.GetAccessorDeclaration &&
                    (a.Body is not null || a.ExpressionBody is not null)))
                return true;
        }
        return false;
    }

    private static List<ISymbol> OnInitMembersInitializing(IPropertySymbol property)
    {
        List<ISymbol> members = [];
        foreach (IMethodSymbol method in property.ContainingType
            .GetMembers(CdacSymbols.DataInitializerMethodName).OfType<IMethodSymbol>())
        {
            foreach (AttributeData attribute in method.GetAttributes())
            {
                if (attribute.AttributeClass?.ToDisplayString() != CdacSymbols.MemberNotNullAttributeMetadataName)
                    continue;
                if (attribute.ConstructorArguments.Any(a => ContainsPropertyName(a, property.Name)))
                {
                    IMethodSymbol implementation = method.PartialImplementationPart ?? method;
                    if (implementation.DeclaringSyntaxReferences.Length > 0)
                        members.Add(implementation);
                    break;
                }
            }
        }
        return members;
    }

    private static bool ContainsPropertyName(TypedConstant argument, string propertyName) =>
        argument.Kind == TypedConstantKind.Array
            ? argument.Values.Any(v => v.Value is string name && name == propertyName)
            : argument.Value is string name && name == propertyName;

    private static List<ISymbol> ConstructorsInitializing(
        CSharpCompilation compilation,
        IPropertySymbol property,
        SymbolEqualityComparer comparer)
    {
        List<ISymbol> constructors = [];
        foreach (IMethodSymbol constructor in property.ContainingType.InstanceConstructors)
        {
            foreach (SyntaxReference reference in constructor.DeclaringSyntaxReferences)
            {
                SyntaxNode syntax = reference.GetSyntax();
                SemanticModel model = compilation.GetSemanticModel(syntax.SyntaxTree);
                if (model.GetOperation(syntax) is not IOperation body)
                    continue;
                if (body.DescendantsAndSelf().OfType<ISimpleAssignmentOperation>().Any(
                    assignment => assignment.Target is IPropertyReferenceOperation target &&
                        comparer.Equals(target.Property.OriginalDefinition, property.OriginalDefinition)))
                {
                    constructors.Add(constructor.OriginalDefinition);
                    break;
                }
            }
        }
        return constructors;
    }

    private static bool TryGetNativeFieldName(IPropertySymbol property, out string nativeName)
    {
        AttributeData? attribute = property.GetAttributes().FirstOrDefault(a =>
            a.AttributeClass?.ToDisplayString() is
                CdacSymbols.FieldAttributeMetadataName or
                CdacSymbols.FieldAddressAttributeMetadataName or
                CdacSymbols.RawOffsetAttributeMetadataName);
        if (attribute is null)
        {
            nativeName = string.Empty;
            return false;
        }

        nativeName = FieldNativeName(attribute, property.Name);
        return true;
    }

    private static string FieldNativeName(AttributeData attribute, string propertyName)
    {
        if (attribute.ConstructorArguments.Length == 1
            && attribute.ConstructorArguments[0].Kind == TypedConstantKind.Array
            && attribute.ConstructorArguments[0].Values is { Length: > 0 } constructorArguments
            && constructorArguments[0].Value is string constructorName)
            return constructorName;

        foreach (KeyValuePair<string, TypedConstant> argument in attribute.NamedArguments)
        {
            if (argument.Key == "Names" && argument.Value.Kind == TypedConstantKind.Array
                && argument.Value.Values is { Length: > 0 } namedArguments
                && namedArguments[0].Value is string namedName)
                return namedName;
        }
        return propertyName;
    }
}
