// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Discovery;

/// <summary>
/// Phase B (part 3): correlates <c>Target.TypeInfo</c> locals/fields/params to their DataType, so
/// a later <c>typeInfo.Fields["nativeName"]</c> raw-string lookup can be attributed to a Data type.
/// Records every <c>x = GetTypeInfo(DataType.X)</c> assignment/declaration across the assembly.
/// </summary>
public sealed class TypeInfoCorrelator
{
    private readonly Dictionary<ISymbol, string> _varToName;

    private TypeInfoCorrelator(Dictionary<ISymbol, string> varToName) => _varToName = varToName;

    /// <summary>
    /// Resolves the DataType name behind a <c>Target.TypeInfo</c> expression: an inline
    /// <c>GetTypeInfo(..)</c> call, or a local/field/param previously assigned from one.
    /// </summary>
    public string? GetTypeInfoDataName(IOperation? expr)
    {
        if (expr is null)
            return null;
        expr = OperationInspector.Unwrap(expr);
        if (OperationInspector.InlineGetTypeInfoName(expr) is string inline)
            return inline;
        return OperationInspector.TargetSymbol(expr) is ISymbol sym && _varToName.TryGetValue(sym, out string? nm)
            ? nm
            : null;
    }

    public static TypeInfoCorrelator Build(Microsoft.CodeAnalysis.Compilation compilation)
    {
        Dictionary<ISymbol, string> varToName = new Dictionary<ISymbol, string>(SymbolEqualityComparer.Default);

        foreach (INamedTypeSymbol type in EnumerateAllTypes(compilation.Assembly.GlobalNamespace))
        foreach (IMethodSymbol method in type.GetMembers().OfType<IMethodSymbol>())
        {
            SyntaxReference? sref = method.DeclaringSyntaxReferences.FirstOrDefault();
            if (sref is null)
                continue;
            SemanticModel model = compilation.GetSemanticModel(sref.SyntaxTree);
            IOperation? body = model.GetOperation(sref.GetSyntax());
            if (body is null)
                continue;

            foreach (IOperation op in body.DescendantsAndSelf())
            {
                switch (op)
                {
                    case ISimpleAssignmentOperation a
                        when OperationInspector.InlineGetTypeInfoName(a.Value) is string n1:
                        if (OperationInspector.TargetSymbol(a.Target) is ISymbol s1)
                            varToName[s1] = n1;
                        break;
                    case IVariableDeclaratorOperation vd
                        when vd.Initializer is not null
                            && OperationInspector.InlineGetTypeInfoName(vd.Initializer.Value) is string n2:
                        varToName[vd.Symbol] = n2;
                        break;
                }
            }
        }

        return new TypeInfoCorrelator(varToName);
    }

    private static IEnumerable<INamedTypeSymbol> EnumerateAllTypes(INamespaceSymbol ns)
    {
        foreach (INamedTypeSymbol t in ns.GetTypeMembers())
            foreach (INamedTypeSymbol nested in EnumerateWithNested(t))
                yield return nested;
        foreach (INamespaceSymbol child in ns.GetNamespaceMembers())
            foreach (INamedTypeSymbol t in EnumerateAllTypes(child))
                yield return t;
    }

    private static IEnumerable<INamedTypeSymbol> EnumerateWithNested(INamedTypeSymbol t)
    {
        yield return t;
        foreach (INamedTypeSymbol nested in t.GetTypeMembers())
            foreach (INamedTypeSymbol n in EnumerateWithNested(nested))
                yield return n;
    }
}
