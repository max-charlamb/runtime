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
    private readonly Dictionary<ISymbol, HashSet<string>> _varToNames;

    private TypeInfoCorrelator(Dictionary<ISymbol, HashSet<string>> varToNames) => _varToNames = varToNames;

    /// <summary>
    /// Resolves the DataType names behind a <c>Target.TypeInfo</c> expression: an inline
    /// <c>GetTypeInfo(..)</c> call, or a local/field/parameter transitively assigned/passed from
    /// one. A set is required for reusable helpers such as <c>DacEnumerableHash</c>, whose
    /// <c>TypeInfo</c> parameter may describe either EETypeHashTable or InstMethodHashTable.
    /// </summary>
    public IReadOnlyCollection<string> GetTypeInfoDataNames(IOperation? expr)
    {
        if (expr is null)
            return [];
        expr = OperationInspector.Unwrap(expr);
        if (OperationInspector.InlineGetTypeInfoName(expr) is string inline)
            return [inline];
        return OperationInspector.TargetSymbol(expr) is ISymbol sym &&
            _varToNames.TryGetValue(sym.OriginalDefinition, out HashSet<string>? names)
                ? names
                : [];
    }

    public static TypeInfoCorrelator Build(Microsoft.CodeAnalysis.Compilation compilation)
    {
        Dictionary<ISymbol, HashSet<string>> varToNames =
            new Dictionary<ISymbol, HashSet<string>>(SymbolEqualityComparer.Default);
        List<IOperation> operations = new List<IOperation>();

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

            operations.AddRange(body.DescendantsAndSelf());
        }

        // Fixed point: seed direct GetTypeInfo assignments, then propagate identities through
        // aliases, method/constructor arguments -> parameters, and parameter -> field assignments.
        // This is intentionally context-insensitive and conservative: a reusable helper parameter
        // accumulates every DataType that can flow to it.
        bool changed;
        do
        {
            changed = false;
            foreach (IOperation op in operations)
            {
                switch (op)
                {
                    case ISimpleAssignmentOperation assignment:
                        if (OperationInspector.TargetSymbol(assignment.Target) is ISymbol assignmentTarget)
                            changed |= AddNames(assignmentTarget, NamesOf(assignment.Value));
                        break;
                    case IVariableDeclaratorOperation declarator when declarator.Initializer is not null:
                        changed |= AddNames(declarator.Symbol, NamesOf(declarator.Initializer.Value));
                        break;
                    case IInvocationOperation invocation:
                        foreach (IArgumentOperation argument in invocation.Arguments)
                        {
                            if (argument.Parameter is IParameterSymbol parameter)
                                changed |= AddNames(parameter, NamesOf(argument.Value));
                        }
                        break;
                    case IObjectCreationOperation creation when creation.Constructor is not null:
                        foreach (IArgumentOperation argument in creation.Arguments)
                        {
                            if (argument.Parameter is IParameterSymbol parameter)
                                changed |= AddNames(parameter, NamesOf(argument.Value));
                        }
                        break;
                }
            }
        }
        while (changed);

        return new TypeInfoCorrelator(varToNames);

        IEnumerable<string> NamesOf(IOperation expression)
        {
            expression = OperationInspector.Unwrap(expression);
            if (OperationInspector.InlineGetTypeInfoName(expression) is string inline)
                return [inline];
            return OperationInspector.TargetSymbol(expression) is ISymbol symbol &&
                varToNames.TryGetValue(symbol.OriginalDefinition, out HashSet<string>? names)
                    ? names
                    : [];
        }

        bool AddNames(ISymbol symbol, IEnumerable<string> names)
        {
            symbol = symbol.OriginalDefinition;
            if (!varToNames.TryGetValue(symbol, out HashSet<string>? set))
                varToNames[symbol] = set = new HashSet<string>(StringComparer.Ordinal);

            bool added = false;
            foreach (string name in names)
                added |= set.Add(name);
            return added;
        }
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
