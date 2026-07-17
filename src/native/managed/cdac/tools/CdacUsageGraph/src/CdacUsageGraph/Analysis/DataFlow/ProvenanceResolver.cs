// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis.DataFlow.Framework;
using CdacUsageGraph.Compilation;
using CdacUsageGraph.Discovery;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// Flow-sensitive cDAC provenance resolver with on-demand interprocedural parameter and field
/// sources. Call arguments feed parameters, interface parameters feed concrete implementations,
/// and field assignments feed later field loads. No whole-assembly symbol/name union is required.
/// </summary>
internal sealed class ProvenanceResolver
{
    private readonly CdacAnalysisWorkspace _workspace;
    private readonly CdacApiSymbols _apiSymbols;
    private readonly Dictionary<ISymbol, FlowResult<DescriptorProvenanceValue, CdacEffect>?> _results =
        new(SymbolEqualityComparer.Default);
    private readonly Dictionary<MethodAnalysisKey, FlowResult<DescriptorProvenanceValue, CdacEffect>?>
        _methodResults = new();
    private readonly HashSet<ISymbol> _activeMethods =
        new(SymbolEqualityComparer.Default);
    private readonly Dictionary<IParameterSymbol, List<IOperation>> _parameterSources =
        new(SymbolEqualityComparer.Default);
    private readonly Dictionary<IParameterSymbol, List<IParameterSymbol>> _parameterAliases =
        new(SymbolEqualityComparer.Default);
    private readonly Dictionary<IFieldSymbol, List<IOperation>> _fieldSources =
        new(SymbolEqualityComparer.Default);
    private readonly Dictionary<ISymbol, FiniteSetValue<DescriptorName>> _symbolValues =
        new(SymbolEqualityComparer.Default);
    private long _cycleVersion;

    public ProvenanceResolver(CdacAnalysisWorkspace workspace)
    {
        _workspace = workspace;
        _apiSymbols = CdacApiSymbols.Build(workspace);
        BuildSourceIndex();
        StabilizeSymbolValues();
    }

    public IReadOnlyCollection<string> GetTypeInfoDataNames(IOperation? expression)
    {
        if (expression is null)
            return [];

        FlowResult<DescriptorProvenanceValue, CdacEffect>? result = GetResult(expression);
        FiniteSetValue<DescriptorName> value = result?.GetValue(expression).TypeInfo ??
            FiniteSetValue<DescriptorName>.Bottom;
        return value.IsUnknown ? [] : value.Values.Select(name => name.Value).ToArray();
    }

    public IReadOnlyCollection<CdacEffect> GetEffects(ISymbol member) =>
        GetResult(NormalizeMember(member))?.Effects ?? [];

    public IReadOnlyCollection<FieldAccessEffect> GetFieldAccessEffects(ISymbol member) =>
        GetEffects(member).OfType<FieldAccessEffect>().ToArray();

    public IReadOnlyCollection<GlobalAccessEffect> GetGlobalAccessEffects(ISymbol member) =>
        GetEffects(member).OfType<GlobalAccessEffect>().ToArray();

    private static ISymbol NormalizeMember(ISymbol member) => member switch
    {
        IPropertySymbol property when property.GetMethod is not null => property.GetMethod,
        _ => member,
    };

    private FlowResult<DescriptorProvenanceValue, CdacEffect>? GetResult(IOperation expression)
    {
        if (!_workspace.TryGetSemanticModel(expression.Syntax.SyntaxTree, out SemanticModel model))
            return null;

        ISymbol? containingSymbol = model.GetEnclosingSymbol(expression.Syntax.SpanStart)?
            .OriginalDefinition;
        if (containingSymbol is null)
            return null;
        return GetResult(NormalizeMember(containingSymbol));
    }

    private FlowResult<DescriptorProvenanceValue, CdacEffect>? GetResult(ISymbol containingSymbol)
    {
        containingSymbol = containingSymbol.OriginalDefinition;
        if (_results.TryGetValue(containingSymbol, out FlowResult<DescriptorProvenanceValue, CdacEffect>? result))
            return result;

        _results.Add(containingSymbol, null);
        result = containingSymbol is IMethodSymbol method
            ? AnalyzeMethod(method, new Dictionary<IParameterSymbol, DescriptorProvenanceValue>(
                SymbolEqualityComparer.Default))
            : null;
        _results[containingSymbol] = result;
        return result;
    }

    private FlowResult<DescriptorProvenanceValue, CdacEffect>? AnalyzeMethod(
        IMethodSymbol method,
        Dictionary<IParameterSymbol, DescriptorProvenanceValue> parameterValues)
    {
        IMethodSymbol implementation = method.PartialImplementationPart ?? method;
        if (_activeMethods.Contains(implementation.OriginalDefinition))
        {
            _cycleVersion++;
            return null;
        }

        MethodAnalysisKey key = new(method, parameterValues);
        if (_methodResults.TryGetValue(key, out FlowResult<DescriptorProvenanceValue, CdacEffect>? cached))
            return cached;

        long cycleVersion = _cycleVersion;
        Dictionary<IParameterSymbol, DescriptorProvenanceValue> implementationParameterValues =
            parameterValues;
        if (!SymbolEqualityComparer.Default.Equals(method, implementation) &&
            parameterValues.Count > 0)
        {
            Dictionary<IParameterSymbol, DescriptorProvenanceValue> remapped =
                new(SymbolEqualityComparer.Default);
            foreach (KeyValuePair<IParameterSymbol, DescriptorProvenanceValue> parameter in parameterValues)
            {
                if (parameter.Key.Ordinal < implementation.Parameters.Length)
                    remapped[implementation.Parameters[parameter.Key.Ordinal]] = parameter.Value;
            }
            implementationParameterValues = remapped;
        }
        _activeMethods.Add(implementation.OriginalDefinition);

        try
        {
            FlowResult<DescriptorProvenanceValue, CdacEffect>? result = null;
            foreach (SyntaxReference syntaxReference in implementation.DeclaringSyntaxReferences)
            {
                if (!_workspace.TryGetSemanticModel(
                        syntaxReference.SyntaxTree,
                        out SemanticModel model))
                    continue;

                ProvenanceDataFlowAnalysis analysis = new(
                    _apiSymbols,
                    ResolveOperationSource,
                    ResolveInvocation,
                    ResolveObjectCreation);
                try
                {
                    result = analysis.Analyze(
                        syntaxReference.GetSyntax(),
                        model,
                        implementationParameterValues);
                    break;
                }
                catch (ArgumentException)
                {
                    continue;
                }
            }

            if (_cycleVersion == cycleVersion)
                _methodResults[key] = result;
            return result;
        }
        finally
        {
            _activeMethods.Remove(implementation.OriginalDefinition);
        }
    }

    private InvocationFlowResult? ResolveInvocation(
        IInvocationOperation invocation,
        IReadOnlyDictionary<int, DescriptorProvenanceValue> arguments)
    {
        IMethodSymbol target = invocation.TargetMethod.ReducedFrom ??
            invocation.TargetMethod;
        if (!_workspace.IsAnalyzable(target))
            return null;
        if (_activeMethods.Contains(target.OriginalDefinition))
        {
            _cycleVersion++;
            return new InvocationFlowResult(
                DescriptorProvenanceValue.Bottom,
                new Dictionary<int, DescriptorProvenanceValue>(),
                []);
        }

        Dictionary<IParameterSymbol, DescriptorProvenanceValue> parameterValues =
            new(SymbolEqualityComparer.Default);
        if (invocation.TargetMethod.ReducedFrom is not null)
        {
            if (invocation.Instance is not null &&
                target.Parameters.Length > 0)
            {
                parameterValues[target.Parameters[0]] =
                    arguments.GetValueOrDefault(-1, DescriptorProvenanceValue.Bottom);
            }
            foreach (KeyValuePair<int, DescriptorProvenanceValue> argument in arguments)
            {
                if (argument.Key >= 0 && argument.Key + 1 < target.Parameters.Length)
                    parameterValues[target.Parameters[argument.Key + 1]] = argument.Value;
            }
        }
        else
        {
            foreach (KeyValuePair<int, DescriptorProvenanceValue> argument in arguments)
            {
                if (argument.Key >= 0 && argument.Key < target.Parameters.Length)
                    parameterValues[target.Parameters[argument.Key]] = argument.Value;
            }
        }

        FlowResult<DescriptorProvenanceValue, CdacEffect>? result = AnalyzeMethod(target, parameterValues);
        if (result is null)
            return null;

        Dictionary<int, DescriptorProvenanceValue> outputs = new();
        foreach (IParameterSymbol parameter in target.Parameters)
        {
            if (parameter.RefKind is RefKind.Out or RefKind.Ref)
            {
                int invocationOrdinal = invocation.TargetMethod.ReducedFrom is null
                    ? parameter.Ordinal
                    : parameter.Ordinal - 1;
                if (invocationOrdinal >= 0)
                    outputs[invocationOrdinal] =
                        result.ExitState[FlowSlot.ForSymbol(parameter)];
            }
        }
        return new InvocationFlowResult(
            result.ReturnValue,
            outputs,
            result.Effects);
    }

    private InvocationFlowResult? ResolveObjectCreation(
        IObjectCreationOperation creation,
        IReadOnlyDictionary<int, DescriptorProvenanceValue> arguments)
    {
        if (creation.Constructor is not IMethodSymbol constructor ||
            !_workspace.IsAnalyzable(constructor))
        {
            return null;
        }
        if (_activeMethods.Contains(constructor.OriginalDefinition))
        {
            _cycleVersion++;
            return null;
        }

        Dictionary<IParameterSymbol, DescriptorProvenanceValue> parameterValues =
            new(SymbolEqualityComparer.Default);
        foreach (KeyValuePair<int, DescriptorProvenanceValue> argument in arguments)
        {
            if (argument.Key >= 0 && argument.Key < constructor.Parameters.Length)
                parameterValues[constructor.Parameters[argument.Key]] = argument.Value;
        }

        FlowResult<DescriptorProvenanceValue, CdacEffect>? result = AnalyzeMethod(constructor, parameterValues);
        return result is null
            ? null
            : new InvocationFlowResult(
                result.ReturnValue,
                new Dictionary<int, DescriptorProvenanceValue>(),
                result.Effects);
    }

    private FiniteSetValue<DescriptorName> ResolveOperationSource(IOperation operation) =>
        OperationInspector.TargetSymbol(operation) is ISymbol symbol &&
        _symbolValues.TryGetValue(symbol.OriginalDefinition, out FiniteSetValue<DescriptorName>? value)
            ? value
            : FiniteSetValue<DescriptorName>.Bottom;

    private void StabilizeSymbolValues()
    {
        HashSet<ISymbol> symbols = new(SymbolEqualityComparer.Default);
        symbols.UnionWith(_parameterSources.Keys);
        symbols.UnionWith(_parameterAliases.Keys);
        symbols.UnionWith(_fieldSources.Keys);

        const int MaxIterations = 100;
        for (int iteration = 0; iteration < MaxIterations; iteration++)
        {
            _results.Clear();
            _methodResults.Clear();
            _activeMethods.Clear();
            _cycleVersion = 0;
            bool changed = false;

            foreach (ISymbol symbol in symbols)
            {
                FiniteSetValue<DescriptorName> value = FiniteSetValue<DescriptorName>.Bottom;
                if (symbol is IParameterSymbol parameter)
                {
                    if (_parameterSources.TryGetValue(parameter, out List<IOperation>? sources))
                    {
                        foreach (IOperation source in sources)
                            value = AddNames(value, GetTypeInfoDataNames(source));
                    }
                    if (_parameterAliases.TryGetValue(parameter, out List<IParameterSymbol>? aliases))
                    {
                        foreach (IParameterSymbol alias in aliases)
                        {
                            if (_symbolValues.TryGetValue(
                                alias.OriginalDefinition,
                                out FiniteSetValue<DescriptorName>? aliasValue))
                                value = value.Join(aliasValue);
                        }
                    }
                }
                else if (symbol is IFieldSymbol field &&
                    _fieldSources.TryGetValue(field, out List<IOperation>? fieldSources))
                {
                    foreach (IOperation source in fieldSources)
                        value = AddNames(value, GetTypeInfoDataNames(source));
                }

                FiniteSetValue<DescriptorName> previous = _symbolValues.GetValueOrDefault(
                    symbol, FiniteSetValue<DescriptorName>.Bottom);
                FiniteSetValue<DescriptorName> joined = previous.Join(value);
                if (!joined.Equals(previous))
                {
                    _symbolValues[symbol] = joined;
                    changed = true;
                }
            }

            if (!changed)
            {
                _activeMethods.Clear();
                return;
            }
        }

        throw new InvalidOperationException(
            $"cDAC provenance did not converge after {MaxIterations} iterations.");
    }

    private void BuildSourceIndex()
    {
        foreach (Microsoft.CodeAnalysis.CSharp.CSharpCompilation compilation
            in _workspace.AnalyzableCompilations)
        {
            foreach (INamedTypeSymbol type in
                compilation.Assembly.GlobalNamespace.EnumerateNamedTypes())
            {
                AddInterfaceParameterAliases(type);
                foreach (IMethodSymbol method in type.GetMembers().OfType<IMethodSymbol>())
                {
                    AddPartialParameterAliases(method);
                    IMethodSymbol implementation = method.PartialImplementationPart ?? method;
                    foreach (SyntaxReference syntaxReference in implementation.DeclaringSyntaxReferences)
                    {
                        if (!_workspace.TryGetSemanticModel(
                                syntaxReference.SyntaxTree,
                                out SemanticModel model))
                            continue;
                        IOperation? root = model.GetOperation(syntaxReference.GetSyntax());
                        if (root is null)
                            continue;

                        foreach (IOperation operation in root.DescendantsAndSelf())
                        {
                            switch (operation)
                            {
                                case IInvocationOperation invocation:
                                    IndexInvocation(invocation);
                                    break;
                                case IObjectCreationOperation creation:
                                    foreach (IArgumentOperation argument in creation.Arguments)
                                    {
                                        if (argument.Parameter is IParameterSymbol parameter &&
                                            _apiSymbols.TypeInfoType.Matches(parameter.Type))
                                            AddSource(
                                                _parameterSources,
                                                (IParameterSymbol)parameter.OriginalDefinition,
                                                argument.Value);
                                    }
                                    break;
                                case ISimpleAssignmentOperation assignment
                                    when OperationInspector.TargetSymbol(assignment.Target)
                                        is IFieldSymbol field &&
                                        _apiSymbols.TypeInfoType.Matches(field.Type):
                                    AddSource(
                                        _fieldSources,
                                        (IFieldSymbol)field.OriginalDefinition,
                                        assignment.Value);
                                    break;
                            }
                        }
                    }
                }
            }
        }
    }

    private void AddPartialParameterAliases(IMethodSymbol method)
    {
        if (method.PartialImplementationPart is not IMethodSymbol implementation)
            return;

        for (int i = 0; i < method.Parameters.Length && i < implementation.Parameters.Length; i++)
        {
            if (!_apiSymbols.TypeInfoType.Matches(implementation.Parameters[i].Type))
                continue;
            AddSource(
                _parameterAliases,
                (IParameterSymbol)implementation.Parameters[i].OriginalDefinition,
                (IParameterSymbol)method.Parameters[i].OriginalDefinition);
        }
    }

    private void IndexInvocation(IInvocationOperation invocation)
    {
        IMethodSymbol target = invocation.TargetMethod.ReducedFrom ??
            invocation.TargetMethod;
        if (invocation.TargetMethod.ReducedFrom is not null &&
            invocation.Instance is not null &&
            target.Parameters.Length > 0 &&
            _apiSymbols.TypeInfoType.Matches(target.Parameters[0].Type))
        {
            AddSource(
                _parameterSources,
                (IParameterSymbol)target.Parameters[0].OriginalDefinition,
                invocation.Instance);
        }
        foreach (IArgumentOperation argument in invocation.Arguments)
        {
            int ordinal = argument.Parameter?.Ordinal ?? -1;
            if (invocation.TargetMethod.ReducedFrom is not null)
                ordinal++;
            if (ordinal >= 0 && ordinal < target.Parameters.Length)
            {
                IParameterSymbol parameter = target.Parameters[ordinal];
                if (_apiSymbols.TypeInfoType.Matches(parameter.Type))
                {
                    AddSource(
                        _parameterSources,
                        (IParameterSymbol)parameter.OriginalDefinition,
                        argument.Value);
                }
            }

            if (argument.Parameter?.RefKind is RefKind.Out or RefKind.Ref &&
                OperationInspector.TargetSymbol(argument.Value) is IFieldSymbol field &&
                _apiSymbols.TypeInfoType.Matches(field.Type))
            {
                AddSource(
                    _fieldSources,
                    (IFieldSymbol)field.OriginalDefinition,
                    argument.Value);
            }
        }
    }

    private void AddInterfaceParameterAliases(INamedTypeSymbol type)
    {
        foreach (INamedTypeSymbol @interface in type.AllInterfaces)
        {
            foreach (IMethodSymbol interfaceMethod in @interface.GetMembers().OfType<IMethodSymbol>())
            {
                if (GenericDispatch.FindInterfaceImplementation(
                        type,
                        interfaceMethod) is not IMethodSymbol implementation)
                    continue;
                for (int i = 0;
                    i < interfaceMethod.Parameters.Length &&
                    i < implementation.Parameters.Length;
                    i++)
                {
                    if (!_apiSymbols.TypeInfoType.Matches(implementation.Parameters[i].Type))
                        continue;
                    AddSource(
                        _parameterAliases,
                        (IParameterSymbol)implementation.Parameters[i].OriginalDefinition,
                        (IParameterSymbol)interfaceMethod.Parameters[i].OriginalDefinition);
                }
            }
        }
    }

    private static void AddSource<TKey, TValue>(
        Dictionary<TKey, List<TValue>> map,
        TKey key,
        TValue value)
        where TKey : notnull
    {
        if (!map.TryGetValue(key, out List<TValue>? values))
            map[key] = values = [];
        values.Add(value);
    }

    private static FiniteSetValue<DescriptorName> AddNames(
        FiniteSetValue<DescriptorName> value,
        IReadOnlyCollection<string> names)
    {
        foreach (string name in names)
            value = value.Join(FiniteSetValue<DescriptorName>.Known(new DescriptorName(name)));
        return value;
    }

    private sealed class MethodAnalysisKey : IEquatable<MethodAnalysisKey>
    {
        private readonly IMethodSymbol _method;
        private readonly DescriptorProvenanceValue[] _parameterValues;
        private readonly int _hashCode;

        public MethodAnalysisKey(
            IMethodSymbol method,
            IReadOnlyDictionary<IParameterSymbol, DescriptorProvenanceValue> parameterValues)
        {
            _method = method.OriginalDefinition;
            _parameterValues = new DescriptorProvenanceValue[method.Parameters.Length];

            HashCode hash = default;
            hash.Add(_method, SymbolEqualityComparer.Default);
            foreach (IParameterSymbol parameter in method.Parameters)
            {
                DescriptorProvenanceValue value = parameterValues.GetValueOrDefault(
                    parameter,
                    DescriptorProvenanceValue.Bottom);
                _parameterValues[parameter.Ordinal] = value;
                hash.Add(value);
            }
            _hashCode = hash.ToHashCode();
        }

        public bool Equals(MethodAnalysisKey? other) =>
            other is not null &&
            _hashCode == other._hashCode &&
            SymbolEqualityComparer.Default.Equals(_method, other._method) &&
            _parameterValues.AsSpan().SequenceEqual(other._parameterValues);

        public override bool Equals(object? obj) => Equals(obj as MethodAnalysisKey);

        public override int GetHashCode() => _hashCode;
    }
}
