// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis;
using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.FlowAnalysis;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// Initial intraprocedural CFG analysis for TypeInfo provenance. This intentionally supports only
/// local/capture flow and inline producer calls; interprocedural summaries are added separately.
/// </summary>
internal sealed class TypeInfoDataFlowAnalysis
{
    private readonly CdacApiSymbols _apiSymbols;
    private readonly Func<IOperation, TypeInfoValue>? _fallback;
    private readonly Func<
        IInvocationOperation,
        IReadOnlyDictionary<int, ProvenanceValue>,
        InvocationFlowResult?>? _invocationResolver;
    private readonly Dictionary<OperationKey, ProvenanceValue> _values = new();
    private readonly HashSet<FieldAccessEffect> _effects = [];
    private ProvenanceValue _returnValue = ProvenanceValue.Bottom;

    public TypeInfoDataFlowAnalysis(
        CdacApiSymbols apiSymbols,
        Func<IOperation, TypeInfoValue>? fallback = null,
        Func<
            IInvocationOperation,
            IReadOnlyDictionary<int, ProvenanceValue>,
            InvocationFlowResult?>? invocationResolver = null)
    {
        _apiSymbols = apiSymbols;
        _fallback = fallback;
        _invocationResolver = invocationResolver;
    }

    public TypeInfoFlowResult Analyze(
        IOperation root,
        IReadOnlyDictionary<IParameterSymbol, ProvenanceValue>? parameterValues = null)
    {
        ControlFlowGraph graph = root switch
        {
            IMethodBodyOperation methodBody => ControlFlowGraph.Create(methodBody),
            IConstructorBodyOperation constructorBody => ControlFlowGraph.Create(constructorBody),
            _ => throw new ArgumentException(
                $"Unsupported CFG root operation '{root.Kind}'.", nameof(root)),
        };
        return Analyze(graph, parameterValues);
    }

    public TypeInfoFlowResult Analyze(
        SyntaxNode declaration,
        SemanticModel semanticModel,
        IReadOnlyDictionary<IParameterSymbol, ProvenanceValue>? parameterValues = null)
    {
        ControlFlowGraph? graph = ControlFlowGraph.Create(declaration, semanticModel);
        if (graph is null)
            throw new ArgumentException(
                $"Could not create a CFG for '{declaration.Kind()}'.",
                nameof(declaration));
        return Analyze(graph, parameterValues);
    }

    private TypeInfoFlowResult Analyze(
        ControlFlowGraph graph,
        IReadOnlyDictionary<IParameterSymbol, ProvenanceValue>? parameterValues)
    {
        _values.Clear();
        _effects.Clear();
        _returnValue = ProvenanceValue.Bottom;
        TypeInfoFlowState?[] inputs = new TypeInfoFlowState?[graph.Blocks.Length];
        TypeInfoFlowState?[] outputs = new TypeInfoFlowState?[graph.Blocks.Length];
        Queue<BasicBlock> worklist = new();
        TypeInfoFlowState entryState = new();
        if (parameterValues is not null)
        {
            foreach (KeyValuePair<IParameterSymbol, ProvenanceValue> parameter in parameterValues)
                entryState[FlowSlot.ForSymbol(parameter.Key)] = parameter.Value;
        }
        inputs[graph.Blocks[0].Ordinal] = entryState;
        worklist.Enqueue(graph.Blocks[0]);

        while (worklist.Count > 0)
        {
            BasicBlock block = worklist.Dequeue();
            TypeInfoFlowState state = inputs[block.Ordinal]!.Clone();
            foreach (IOperation operation in block.Operations)
                Evaluate(operation, state);
            ProvenanceValue branchValue = block.BranchValue is not null
                ? Evaluate(block.BranchValue, state)
                : ProvenanceValue.Bottom;
            if (block.FallThroughSuccessor?.Semantics == ControlFlowBranchSemantics.Return ||
                block.ConditionalSuccessor?.Semantics == ControlFlowBranchSemantics.Return)
                _returnValue = _returnValue.Join(branchValue);

            if (outputs[block.Ordinal]?.Equals(state) == true)
                continue;
            outputs[block.Ordinal] = state;

            Propagate(block.FallThroughSuccessor, state);
            Propagate(block.ConditionalSuccessor, state);
        }

        BasicBlock exitBlock = graph.Blocks[^1];
        TypeInfoFlowState exitState =
            inputs[exitBlock.Ordinal]?.Clone() ?? new TypeInfoFlowState();
        return new TypeInfoFlowResult(
            new Dictionary<OperationKey, ProvenanceValue>(_values),
            _effects.ToArray(),
            _returnValue,
            exitState);

        void Propagate(ControlFlowBranch? branch, TypeInfoFlowState state)
        {
            if (branch?.Destination is not BasicBlock destination)
                return;

            if (inputs[destination.Ordinal] is null)
            {
                inputs[destination.Ordinal] = state.Clone();
                worklist.Enqueue(destination);
            }
            else if (inputs[destination.Ordinal]!.JoinFrom(state))
            {
                worklist.Enqueue(destination);
            }
        }
    }

    private ProvenanceValue Evaluate(IOperation operation, TypeInfoFlowState state)
    {
        ProvenanceValue result = operation switch
        {
            IConversionOperation conversion => Evaluate(conversion.Operand, state),
            IArgumentOperation argument => Evaluate(argument.Value, state),
            ILiteralOperation literal
                when literal.ConstantValue is { HasValue: true, Value: string text } =>
                    ProvenanceValue.FromString(StringValue.Known(text)),
            ILocalReferenceOperation local => state[FlowSlot.ForSymbol(local.Local)],
            IParameterReferenceOperation parameter => state[FlowSlot.ForSymbol(parameter.Parameter)],
            IFieldReferenceOperation field
                when field.ConstantValue is { HasValue: true, Value: string text } =>
                    ProvenanceValue.FromString(StringValue.Known(text)),
            IFieldReferenceOperation field
                when field.Field.ContainingType?.TypeKind == TypeKind.Enum =>
                    ProvenanceValue.FromString(StringValue.Known(field.Field.Name)),
            IFieldReferenceOperation field => state[FlowSlot.ForSymbol(field.Field)],
            IFlowCaptureReferenceOperation capture => state[FlowSlot.ForCapture(capture.Id)],
            IFlowCaptureOperation capture => Assign(
                FlowSlot.ForCapture(capture.Id),
                Evaluate(capture.Value, state),
                state),
            ISimpleAssignmentOperation assignment => Assign(
                TargetSlot(assignment.Target),
                Evaluate(assignment.Value, state),
                state),
            IVariableDeclaratorOperation declarator when declarator.Initializer is not null => Assign(
                FlowSlot.ForSymbol(declarator.Symbol),
                Evaluate(declarator.Initializer.Value, state),
                state),
            IPropertyReferenceOperation property => EvaluateProperty(property, state),
            IBinaryOperation binary => EvaluateBinary(binary, state),
            IInvocationOperation invocation => EvaluateInvocation(invocation, state),
            _ => EvaluateChildren(operation, state),
        };

        if (result.TypeInfo.Equals(TypeInfoValue.Bottom) && _fallback is not null)
        {
            TypeInfoValue fallback = _fallback(operation);
            if (!fallback.Equals(TypeInfoValue.Bottom))
                result = result.Join(ProvenanceValue.FromTypeInfo(fallback));
        }

        Remember(operation, result);
        return result;
    }

    private ProvenanceValue EvaluateInvocation(
        IInvocationOperation invocation,
        TypeInfoFlowState state)
    {
        ProvenanceValue instance = invocation.Instance is null
            ? ProvenanceValue.Bottom
            : Evaluate(invocation.Instance, state);
        if (!instance.FieldsOwner.Equals(TypeInfoValue.Bottom) &&
            invocation.TargetMethod.Name is "ContainsKey" or "TryGetValue")
        {
            ProvenanceValue fieldName = EvaluateArgument(invocation, 0, state);
            FieldValue fields = CreateFields(instance.FieldsOwner, fieldName.Strings);
            RecordFields(instance.FieldsOwner, fieldName.Strings, UsageKind.OffsetLookup);

            IArgumentOperation? outArgument = invocation.Arguments.FirstOrDefault(
                argument => argument.Parameter?.RefKind == RefKind.Out);
            if (outArgument is not null)
            {
                _ = Assign(
                    TargetSlot(outArgument.Value),
                    ProvenanceValue.FromFields(fields),
                    state);
            }
            return ProvenanceValue.Bottom;
        }

        if (_apiSymbols.DataTypeToName?.Matches(invocation.TargetMethod) == true)
        {
            if (invocation.TargetMethod.ReducedFrom is not null && invocation.Instance is not null)
                return ProvenanceValue.FromString(instance.Strings);

            IArgumentOperation? valueArgument = invocation.Arguments.FirstOrDefault(
                argument => argument.Parameter?.Ordinal == 0);
            return valueArgument is null
                ? ProvenanceValue.FromString(StringValue.Unknown)
                : ProvenanceValue.FromString(Evaluate(valueArgument.Value, state).Strings);
        }

        if (_apiSymbols.TargetGetTypeInfo.Matches(invocation.TargetMethod) ||
            _apiSymbols.ManagedGetTypeInfo.Matches(invocation.TargetMethod))
        {
            return ProvenanceValue.FromTypeInfo(TypeInfoFromStrings(
                EvaluateArgument(invocation, 0, state).Strings));
        }

        if (_apiSymbols.TargetTryGetTypeInfo.Matches(invocation.TargetMethod) ||
            _apiSymbols.ManagedTryGetTypeInfo.Matches(invocation.TargetMethod))
        {
            ProvenanceValue value = ProvenanceValue.FromTypeInfo(TypeInfoFromStrings(
                EvaluateArgument(invocation, 0, state).Strings));
            IArgumentOperation outArgument = invocation.Arguments.Single(
                argument => argument.Parameter?.RefKind == RefKind.Out);
            _ = Assign(TargetSlot(outArgument.Value), value, state);
            Remember(outArgument.Value, value);
            return ProvenanceValue.Bottom;
        }

        Dictionary<int, ProvenanceValue> arguments = invocation.Arguments.ToDictionary(
            argument => argument.Parameter?.Ordinal ?? -1,
            argument => Evaluate(argument.Value, state));
        if (invocation.TargetMethod.ReducedFrom is not null &&
            invocation.Instance is not null)
        {
            arguments[-1] = Evaluate(invocation.Instance, state);
        }
        InvocationFlowResult? invocationResult = _invocationResolver?.Invoke(
            invocation, arguments);
        if (invocationResult is not null)
        {
            _effects.UnionWith(invocationResult.Effects);
            foreach (KeyValuePair<int, ProvenanceValue> output in invocationResult.OutRefValues)
            {
                IArgumentOperation? argument = invocation.Arguments.FirstOrDefault(
                    candidate => candidate.Parameter?.Ordinal == output.Key);
                if (argument is not null)
                {
                    _ = Assign(TargetSlot(argument.Value), output.Value, state);
                    Remember(argument.Value, output.Value);
                }
            }
            return invocationResult.ReturnValue;
        }

        if (arguments.TryGetValue(0, out ProvenanceValue? address))
        {
            if (_apiSymbols.TargetReadMethods.Matches(invocation.TargetMethod))
                RecordAddressEffects(address, UsageKind.Read);
            else if (_apiSymbols.DataCacheGetOrAdd?.Matches(invocation.TargetMethod) == true)
                RecordAddressEffects(address, UsageKind.Read);
            else if (_apiSymbols.TargetWriteMethods.Matches(invocation.TargetMethod))
                RecordAddressEffects(address, UsageKind.Write);
        }
        return ProvenanceValue.Bottom;
    }

    private ProvenanceValue EvaluateArgument(
        IInvocationOperation invocation,
        int parameterOrdinal,
        TypeInfoFlowState state)
    {
        IArgumentOperation? argument = invocation.Arguments.FirstOrDefault(
            candidate => candidate.Parameter?.Ordinal == parameterOrdinal);
        return argument is null
            ? ProvenanceValue.Bottom
            : Evaluate(argument.Value, state);
    }

    private static TypeInfoValue TypeInfoFromStrings(StringValue strings)
    {
        if (strings.IsUnknown)
            return TypeInfoValue.Unknown;

        TypeInfoValue value = TypeInfoValue.Bottom;
        foreach (string name in strings.Values)
            value = value.Join(TypeInfoValue.Known(name));
        return value;
    }

    private ProvenanceValue EvaluateProperty(
        IPropertyReferenceOperation property,
        TypeInfoFlowState state)
    {
        if (_apiSymbols.TypeInfoFields.Matches(property.Property))
            return ProvenanceValue.FromFieldsOwner(Evaluate(property.Instance!, state).TypeInfo);

        if (_apiSymbols.TypeInfoSize.Matches(property.Property))
        {
            TypeInfoValue typeInfo = Evaluate(property.Instance!, state).TypeInfo;
            RecordFields(typeInfo, StringValue.Known("Size"), UsageKind.Read);
            return ProvenanceValue.Bottom;
        }

        if (_apiSymbols.FieldInfoOffset.Matches(property.Property))
            return ProvenanceValue.FromOffsets(Evaluate(property.Instance!, state).Fields);

        if (property.Property.IsIndexer)
        {
            ProvenanceValue instance = Evaluate(property.Instance!, state);
            if (!instance.FieldsOwner.Equals(TypeInfoValue.Bottom))
            {
                StringValue names = StringValue.Bottom;
                foreach (IArgumentOperation argument in property.Arguments)
                    names = names.Join(Evaluate(argument.Value, state).Strings);
                RecordFields(instance.FieldsOwner, names, UsageKind.OffsetLookup);
                return ProvenanceValue.FromFields(CreateFields(instance.FieldsOwner, names));
            }
        }

        return EvaluateChildren(property, state);
    }

    private ProvenanceValue EvaluateBinary(
        IBinaryOperation binary,
        TypeInfoFlowState state)
    {
        ProvenanceValue left = Evaluate(binary.LeftOperand, state);
        ProvenanceValue right = Evaluate(binary.RightOperand, state);
        FieldValue fields = left.Addresses
            .Join(right.Addresses)
            .Join(left.Offsets)
            .Join(right.Offsets);
        return fields.Equals(FieldValue.Bottom)
            ? ProvenanceValue.Bottom
            : ProvenanceValue.FromAddresses(fields);
    }

    private ProvenanceValue EvaluateChildren(IOperation operation, TypeInfoFlowState state)
    {
        foreach (IOperation child in operation.ChildOperations)
            Evaluate(child, state);
        return ProvenanceValue.Bottom;
    }

    private static ProvenanceValue Assign(
        FlowSlot? target,
        ProvenanceValue value,
        TypeInfoFlowState state)
    {
        if (target.HasValue)
            state[target.Value] = value;
        return value;
    }

    private static FlowSlot? TargetSlot(IOperation operation) =>
        OperationInspector.Unwrap(operation) switch
        {
            ILocalReferenceOperation local => FlowSlot.ForSymbol(local.Local),
            IParameterReferenceOperation parameter => FlowSlot.ForSymbol(parameter.Parameter),
            IFieldReferenceOperation field => FlowSlot.ForSymbol(field.Field),
            IFlowCaptureReferenceOperation capture => FlowSlot.ForCapture(capture.Id),
            IDeclarationExpressionOperation declaration => TargetSlot(declaration.Expression),
            _ => null,
        };

    private void RecordAddressEffects(ProvenanceValue address, UsageKind usage)
    {
        foreach (FieldIdentity field in address.Addresses.Fields)
            _effects.Add(new FieldAccessEffect(field, usage));
    }

    private void RecordFields(TypeInfoValue typeInfo, StringValue names, UsageKind usage)
    {
        foreach (FieldIdentity field in CreateFields(typeInfo, names).Fields)
            _effects.Add(new FieldAccessEffect(field, usage));
    }

    private static FieldValue CreateFields(TypeInfoValue typeInfo, StringValue names)
    {
        if (typeInfo.IsUnknown || names.IsUnknown)
            return FieldValue.Unknown;

        return FieldValue.Known(
            from typeName in typeInfo.Names
            from fieldName in names.Values
            select new FieldIdentity(typeName, fieldName));
    }

    private void Remember(IOperation operation, ProvenanceValue value)
    {
        OperationKey key = OperationKey.Create(operation);
        if (_values.TryGetValue(key, out ProvenanceValue? current))
            _values[key] = current.Join(value);
        else
            _values[key] = value;
    }
}
