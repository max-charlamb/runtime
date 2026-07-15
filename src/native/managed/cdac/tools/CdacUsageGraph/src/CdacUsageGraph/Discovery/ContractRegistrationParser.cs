// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Discovery;

/// <summary>
/// Phase B (part 2): parses <c>CoreCLRContracts.Register&lt;IContract&gt;("cN", t =&gt; new Impl(t))</c>
/// into the authoritative (interface, version) -&gt; implementation map. Handles multi-impl lambdas
/// (e.g. the <c>IGCInfo</c> arch switch) by taking every constructed in-assembly type.
/// </summary>
internal static class ContractRegistrationParser
{
    public static IReadOnlyList<ContractRegistration> Parse(Microsoft.CodeAnalysis.Compilation compilation)
    {
        SymbolEqualityComparer comparer = SymbolEqualityComparer.Default;
        List<ContractRegistration> registrations = new List<ContractRegistration>();
        INamedTypeSymbol? contractRegistry = compilation.GetTypeByMetadataName(
            CdacSymbols.ContractRegistryMetadataName);
        INamedTypeSymbol? iContract = compilation.GetTypeByMetadataName(
            CdacSymbols.IContractMetadataName);

        INamedTypeSymbol? coreContracts = compilation.GetTypeByMetadataName(
            CdacSymbols.CoreCLRContractsMetadataName);
        if (coreContracts is not null && contractRegistry is not null && iContract is not null)
        {
            foreach (IMethodSymbol reg in coreContracts.GetMembers().OfType<IMethodSymbol>())
            {
                SyntaxReference? sref = reg.DeclaringSyntaxReferences.FirstOrDefault();
                if (sref is null)
                    continue;
                SyntaxNode syntax = sref.GetSyntax();
                SemanticModel model = compilation.GetSemanticModel(syntax.SyntaxTree);
                IOperation? op = model.GetOperation(syntax);
                if (op is null)
                    continue;

                foreach (IInvocationOperation inv in op.DescendantsAndSelf().OfType<IInvocationOperation>())
                {
                    if (inv.TargetMethod.Name != CdacSymbols.ContractRegistrationMethodName)
                        continue;
                    if (inv.Instance?.Type is not INamedTypeSymbol receiver ||
                        !IsOrInheritsFrom(receiver, contractRegistry, comparer))
                        continue;
                    if (inv.TargetMethod.TypeArguments.Length != 1)
                        continue;
                    if (inv.TargetMethod.TypeArguments[0] is not INamedTypeSymbol iface)
                        continue;
                    if (!ImplementsOrEquals(iface, iContract, comparer))
                        continue;

                    string? version = inv.Arguments
                        .Select(a => a.Value)
                        .Select(v => v is IConversionOperation c ? c.Operand : v)
                        .Select(v => v.ConstantValue is { HasValue: true, Value: string s } ? s : null)
                        .FirstOrDefault(s => s is not null);
                    if (version is null)
                        continue;

                    foreach (IObjectCreationOperation create in inv.Descendants().OfType<IObjectCreationOperation>())
                    {
                        if (create.Type is INamedTypeSymbol impl &&
                            comparer.Equals(impl.ContainingAssembly, compilation.Assembly) &&
                            ImplementsOrEquals(impl, iface, comparer) &&
                            ImplementsOrEquals(impl, iContract, comparer))
                        {
                            registrations.Add(new ContractRegistration(iface.Name, version, impl));
                        }
                    }
                }
            }
        }

        // Deduplicate on (contract, version, impl name), preserving source order.
        return registrations
            .GroupBy(r => (r.Contract, r.Version, r.Impl.Name))
            .Select(g => g.First())
            .ToList();
    }

    private static bool IsOrInheritsFrom(
        INamedTypeSymbol type,
        INamedTypeSymbol expectedBase,
        SymbolEqualityComparer comparer)
    {
        for (INamedTypeSymbol? current = type; current is not null; current = current.BaseType)
        {
            if (comparer.Equals(current.OriginalDefinition, expectedBase.OriginalDefinition))
                return true;
        }
        return false;
    }

    private static bool ImplementsOrEquals(
        INamedTypeSymbol type,
        INamedTypeSymbol expectedInterface,
        SymbolEqualityComparer comparer) =>
        comparer.Equals(type.OriginalDefinition, expectedInterface.OriginalDefinition) ||
        type.AllInterfaces.Any(i =>
            comparer.Equals(i.OriginalDefinition, expectedInterface.OriginalDefinition));
}
