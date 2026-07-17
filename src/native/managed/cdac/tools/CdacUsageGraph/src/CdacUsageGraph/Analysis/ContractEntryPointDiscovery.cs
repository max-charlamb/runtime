// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Analysis;

/// <summary>Discovers the methods and initializers through which a registered contract can run.</summary>
internal static class ContractEntryPointDiscovery
{
    public static IReadOnlyCollection<ISymbol> Discover(ContractRegistration registration)
    {
        HashSet<ISymbol> entries = new(SymbolEqualityComparer.Default);
        if (registration.Constructor is not null)
            entries.Add(registration.Constructor.OriginalDefinition);
        else
            entries.UnionWith(registration.Impl.InstanceConstructors
                .Where(constructor => constructor.DeclaringSyntaxReferences.Length > 0)
                .Select(constructor => constructor.OriginalDefinition));

        AddInitializers(registration.Impl, entries);

        if (registration.Interface is null)
        {
            entries.UnionWith(registration.Impl.GetMembers().OfType<IMethodSymbol>()
                .Where(IsWalkableMethod)
                .Select(method => method.OriginalDefinition));
            return entries;
        }

        foreach (INamedTypeSymbol @interface in registration.Interface.AllInterfaces
            .Prepend(registration.Interface))
        {
            foreach (ISymbol member in @interface.GetMembers())
            {
                switch (member)
                {
                    case IMethodSymbol method:
                        AddImplementation(registration.Impl, method, entries);
                        break;
                    case IPropertySymbol property:
                        if (property.GetMethod is not null)
                            AddImplementation(registration.Impl, property.GetMethod, entries);
                        if (property.SetMethod is not null)
                            AddImplementation(registration.Impl, property.SetMethod, entries);
                        break;
                }
            }
        }

        return entries;
    }

    public static IReadOnlyCollection<ISymbol> DiscoverConstruction(
        INamedTypeSymbol type,
        IMethodSymbol? constructor)
    {
        HashSet<ISymbol> entries = new(SymbolEqualityComparer.Default);
        if (constructor is not null)
            entries.Add(constructor.OriginalDefinition);
        AddInitializers(type, entries);
        return entries;
    }

    private static void AddImplementation(
        INamedTypeSymbol implementationType,
        ISymbol interfaceMember,
        HashSet<ISymbol> entries)
    {
        if (implementationType.FindImplementationForInterfaceMember(interfaceMember)
                is IMethodSymbol implementation &&
            SymbolEqualityComparer.Default.Equals(
                implementation.ContainingAssembly,
                implementationType.ContainingAssembly))
        {
            IMethodSymbol entryPoint =
                GenericDispatch.FindVirtualImplementation(
                    implementationType,
                    implementation) ?? implementation;
            entries.Add(entryPoint.OriginalDefinition);
        }
    }

    private static void AddInitializers(INamedTypeSymbol type, HashSet<ISymbol> entries)
    {
        for (INamedTypeSymbol? current = type;
            current is not null && SymbolEqualityComparer.Default.Equals(
                current.ContainingAssembly,
                type.ContainingAssembly);
            current = current.BaseType)
        {
            foreach (ISymbol member in current.OriginalDefinition.GetMembers())
            {
                if (member is IFieldSymbol { IsConst: false, IsImplicitlyDeclared: false } or
                    IPropertySymbol)
                    entries.Add(member);
            }
        }
    }

    private static bool IsWalkableMethod(IMethodSymbol method) =>
        method.MethodKind is MethodKind.Ordinary or MethodKind.Constructor
            or MethodKind.ExplicitInterfaceImplementation or MethodKind.PropertyGet
            or MethodKind.PropertySet;
}
