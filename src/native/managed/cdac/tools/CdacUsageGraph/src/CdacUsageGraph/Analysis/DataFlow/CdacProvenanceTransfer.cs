// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis.DataFlow.Framework;
using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.FlowAnalysis;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// The cDAC provenance transfer function: evaluates one operation's <see cref="DescriptorProvenanceValue"/>
/// against the current <see cref="FlowState{TValue}"/>, recognizing the primitive cDAC TypeInfo/global
/// APIs (<see cref="CdacApiSymbols"/>) and recording <see cref="CdacEffect"/>s for field and global
/// accesses. All CFG traversal, worklist, and join behavior lives in
/// <see cref="ForwardControlFlowSolver{TValue, TEffect}"/>; this type only knows what individual
/// operations mean.
/// </summary>
internal sealed class CdacProvenanceTransfer : IOperationTransfer<DescriptorProvenanceValue, CdacEffect>
{
    private readonly CdacApiSymbols _apiSymbols;
    private readonly Func<IOperation, FiniteSetValue<DescriptorName>>? _fallback;
    private readonly Func<
        IInvocationOperation,
        IReadOnlyDictionary<int, DescriptorProvenanceValue>,
        InvocationFlowResult?>? _invocationResolver;
    private readonly Func<
        IObjectCreationOperation,
        IReadOnlyDictionary<int, DescriptorProvenanceValue>,
        InvocationFlowResult?>? _objectCreationResolver;

    public CdacProvenanceTransfer(
        CdacApiSymbols apiSymbols,
        Func<IOperation, FiniteSetValue<DescriptorName>>? fallback,
        Func<
            IInvocationOperation,
            IReadOnlyDictionary<int, DescriptorProvenanceValue>,
            InvocationFlowResult?>? invocationResolver,
        Func<
            IObjectCreationOperation,
            IReadOnlyDictionary<int, DescriptorProvenanceValue>,
            InvocationFlowResult?>? objectCreationResolver)
    {
        _apiSymbols = apiSymbols;
        _fallback = fallback;
        _invocationResolver = invocationResolver;
        _objectCreationResolver = objectCreationResolver;
    }

    public DescriptorProvenanceValue Evaluate(
        IOperation operation,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        DescriptorProvenanceValue result = operation switch
        {
            IConversionOperation conversion => Evaluate(conversion.Operand, state, sink),
            IArgumentOperation argument => Evaluate(argument.Value, state, sink),
            ILiteralOperation literal
                when literal.ConstantValue is { HasValue: true, Value: string text } =>
                    DescriptorProvenanceValue.FromString(FiniteSetValue<string>.Known(text)),
            ILocalReferenceOperation local => state[FlowSlot.ForSymbol(local.Local)],
            IParameterReferenceOperation parameter => state[FlowSlot.ForSymbol(parameter.Parameter)],
            IFieldReferenceOperation field
                when field.ConstantValue is { HasValue: true, Value: string text } =>
                    DescriptorProvenanceValue.FromString(FiniteSetValue<string>.Known(text)),
            IFieldReferenceOperation field
                when field.Field.ContainingType?.TypeKind == TypeKind.Enum =>
                    DescriptorProvenanceValue.FromString(FiniteSetValue<string>.Known(field.Field.Name)),
            IFieldReferenceOperation field => state[FlowSlot.ForSymbol(field.Field)],
            IFlowCaptureReferenceOperation capture => state[FlowSlot.ForCapture(capture.Id)],
            IFlowCaptureOperation capture => Assign(
                FlowSlot.ForCapture(capture.Id),
                Evaluate(capture.Value, state, sink),
                state),
            ISimpleAssignmentOperation assignment => Assign(
                TargetSlot(assignment.Target),
                Evaluate(assignment.Value, state, sink),
                state),
            IVariableDeclaratorOperation declarator when declarator.Initializer is not null => Assign(
                FlowSlot.ForSymbol(declarator.Symbol),
                Evaluate(declarator.Initializer.Value, state, sink),
                state),
            ISwitchExpressionOperation switchExpression =>
                EvaluateSwitchExpression(switchExpression, state, sink),
            IPropertyReferenceOperation property => EvaluateProperty(property, state, sink),
            IBinaryOperation binary => EvaluateBinary(binary, state, sink),
            IInvocationOperation invocation => EvaluateInvocation(invocation, state, sink),
            IObjectCreationOperation creation => EvaluateObjectCreation(creation, state, sink),
            _ => EvaluateChildren(operation, state, sink),
        };

        if (result.TypeInfo.Equals(FiniteSetValue<DescriptorName>.Bottom) && _fallback is not null)
        {
            FiniteSetValue<DescriptorName> fallback = _fallback(operation);
            if (!fallback.Equals(FiniteSetValue<DescriptorName>.Bottom))
                result = result.Join(DescriptorProvenanceValue.FromTypeInfo(fallback));
        }

        sink.Record(operation, result);
        return result;
    }

    private DescriptorProvenanceValue EvaluateInvocation(
        IInvocationOperation invocation,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        DescriptorProvenanceValue instance = invocation.Instance is null
            ? DescriptorProvenanceValue.Bottom
            : Evaluate(invocation.Instance, state, sink);
        if (!instance.FieldsOwner.Equals(FiniteSetValue<DescriptorName>.Bottom) &&
            invocation.TargetMethod.Name is "ContainsKey" or "TryGetValue")
        {
            DescriptorProvenanceValue fieldName = EvaluateArgument(invocation, 0, state, sink);
            FiniteSetValue<FieldIdentity> fields = CreateFields(instance.FieldsOwner, fieldName.Strings);
            RecordFields(instance.FieldsOwner, fieldName.Strings, UsageKind.OffsetLookup, sink);

            IArgumentOperation? outArgument = invocation.Arguments.FirstOrDefault(
                argument => argument.Parameter?.RefKind == RefKind.Out);
            if (outArgument is not null)
            {
                _ = Assign(
                    TargetSlot(outArgument.Value),
                    DescriptorProvenanceValue.FromFields(fields),
                    state);
            }
            return DescriptorProvenanceValue.Bottom;
        }

        if (TryRecordGlobalAccess(invocation, state, sink))
            return DescriptorProvenanceValue.Bottom;

        if (_apiSymbols.DataTypeToName?.Matches(invocation.TargetMethod) == true)
        {
            if (invocation.TargetMethod.ReducedFrom is not null && invocation.Instance is not null)
                return DescriptorProvenanceValue.FromString(instance.Strings);

            IArgumentOperation? valueArgument = invocation.Arguments.FirstOrDefault(
                argument => argument.Parameter?.Ordinal == 0);
            return valueArgument is null
                ? DescriptorProvenanceValue.FromString(FiniteSetValue<string>.Unknown)
                : DescriptorProvenanceValue.FromString(Evaluate(valueArgument.Value, state, sink).Strings);
        }

        if (invocation.TargetMethod.Name == nameof(ToString) &&
            invocation.Arguments.Length == 0 &&
            invocation.Instance?.Type?.TypeKind == TypeKind.Enum)
        {
            return DescriptorProvenanceValue.FromString(
                Evaluate(invocation.Instance, state, sink).Strings);
        }

        if (_apiSymbols.TargetGetTypeInfo.Matches(invocation.TargetMethod) ||
            _apiSymbols.ManagedGetTypeInfo.Matches(invocation.TargetMethod))
        {
            return DescriptorProvenanceValue.FromTypeInfo(DescriptorNamesFromStrings(
                EvaluateArgument(invocation, 0, state, sink).Strings));
        }

        if (_apiSymbols.TargetTryGetTypeInfo.Matches(invocation.TargetMethod) ||
            _apiSymbols.ManagedTryGetTypeInfo.Matches(invocation.TargetMethod))
        {
            DescriptorProvenanceValue value = DescriptorProvenanceValue.FromTypeInfo(DescriptorNamesFromStrings(
                EvaluateArgument(invocation, 0, state, sink).Strings));
            IArgumentOperation outArgument = invocation.Arguments.Single(
                argument => argument.Parameter?.RefKind == RefKind.Out);
            _ = Assign(TargetSlot(outArgument.Value), value, state);
            sink.Record(outArgument.Value, value);
            return DescriptorProvenanceValue.Bottom;
        }

        Dictionary<int, DescriptorProvenanceValue> arguments = invocation.Arguments.ToDictionary(
            argument => argument.Parameter?.Ordinal ?? -1,
            argument => Evaluate(argument.Value, state, sink));
        if (invocation.TargetMethod.ReducedFrom is not null &&
            invocation.Instance is not null)
        {
            arguments[-1] = Evaluate(invocation.Instance, state, sink);
        }
        InvocationFlowResult? invocationResult = _invocationResolver?.Invoke(
            invocation, arguments);
        if (invocationResult is not null)
        {
            sink.AddEffects(invocationResult.Effects);
            foreach (KeyValuePair<int, DescriptorProvenanceValue> output in invocationResult.OutRefValues)
            {
                IArgumentOperation? argument = invocation.Arguments.FirstOrDefault(
                    candidate => candidate.Parameter?.Ordinal == output.Key);
                if (argument is not null)
                {
                    _ = Assign(TargetSlot(argument.Value), output.Value, state);
                    sink.Record(argument.Value, output.Value);
                }
            }
            return invocationResult.ReturnValue;
        }

        if (arguments.TryGetValue(0, out DescriptorProvenanceValue? address))
        {
            if (_apiSymbols.TargetReadMethods.Matches(invocation.TargetMethod))
                RecordAddressEffects(address, UsageKind.Read, sink);
            else if (_apiSymbols.DataCacheGetOrAdd?.Matches(invocation.TargetMethod) == true)
                RecordAddressEffects(address, UsageKind.Read, sink);
            else if (_apiSymbols.TargetWriteMethods.Matches(invocation.TargetMethod))
                RecordAddressEffects(address, UsageKind.Write, sink);
        }
        return DescriptorProvenanceValue.Bottom;
    }

    private DescriptorProvenanceValue EvaluateObjectCreation(
        IObjectCreationOperation creation,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        Dictionary<int, DescriptorProvenanceValue> arguments = creation.Arguments.ToDictionary(
            argument => argument.Parameter?.Ordinal ?? -1,
            argument => Evaluate(argument.Value, state, sink));
        InvocationFlowResult? result = _objectCreationResolver?.Invoke(
            creation,
            arguments);
        if (result is null)
            return DescriptorProvenanceValue.Bottom;

        sink.AddEffects(result.Effects);
        return result.ReturnValue;
    }

    private bool TryRecordGlobalAccess(
        IInvocationOperation invocation,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        string? type = null;
        bool isOptional = false;
        if (_apiSymbols.TargetReadGlobalPointer.Matches(invocation.TargetMethod))
        {
            type = "pointer";
        }
        else if (_apiSymbols.TargetTryReadGlobalPointer.Matches(invocation.TargetMethod))
        {
            type = "pointer";
            isOptional = true;
        }
        else if (_apiSymbols.TargetReadGlobalString.Matches(invocation.TargetMethod))
        {
            type = "string";
        }
        else if (_apiSymbols.TargetTryReadGlobalString.Matches(invocation.TargetMethod))
        {
            type = "string";
            isOptional = true;
        }
        else if (_apiSymbols.TargetReadGlobal.Matches(invocation.TargetMethod))
        {
            type = NativeTypeName.FromType(invocation.TargetMethod.TypeArguments[0]);
        }
        else if (_apiSymbols.TargetTryReadGlobal.Matches(invocation.TargetMethod))
        {
            type = NativeTypeName.FromType(invocation.TargetMethod.TypeArguments[0]);
            isOptional = true;
        }

        if (type is null)
            return false;

        FiniteSetValue<string> names = EvaluateArgument(invocation, 0, state, sink).Strings;
        HashSet<string> resolvedNames = new(names.Values, StringComparer.Ordinal);
        if (resolvedNames.Count == 0)
        {
            IArgumentOperation? nameArgument = invocation.Arguments.FirstOrDefault(
                argument => argument.Parameter?.Ordinal == 0);
            if (nameArgument is not null)
            {
                resolvedNames.UnionWith(
                    DescribeGlobalNamePatterns(nameArgument.Value));
            }
        }
        foreach (string name in resolvedNames)
            sink.AddEffect(new GlobalAccessEffect(name, type, isOptional));
        return true;
    }

    private DescriptorProvenanceValue EvaluateSwitchExpression(
        ISwitchExpressionOperation switchExpression,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        FiniteSetValue<string> values = FiniteSetValue<string>.Bottom;
        _ = Evaluate(switchExpression.Value, state, sink);
        foreach (ISwitchExpressionArmOperation arm in switchExpression.Arms)
            values = values.Join(Evaluate(arm.Value, state, sink).Strings);
        return DescriptorProvenanceValue.FromString(values);
    }

    private static string[] DescribeGlobalNamePatterns(
        IOperation operation)
    {
        operation = OperationInspector.Unwrap(operation);
        if (operation.ConstantValue is { HasValue: true, Value: string text })
            return [text];

        if (operation is ILocalReferenceOperation local &&
            local.Type?.SpecialType == SpecialType.System_String)
        {
            return [$"<{local.Local.Name}>"];
        }

        if (operation is IParameterReferenceOperation parameter &&
            parameter.Type?.SpecialType == SpecialType.System_String)
        {
            return [$"<{parameter.Parameter.Name}>"];
        }

        if (operation is IInvocationOperation
            {
                TargetMethod.Name: nameof(ToString),
                Arguments.Length: 0,
                Instance.Type.TypeKind: TypeKind.Enum,
            } invocation)
        {
            return [$"<{invocation.Instance!.Type!.Name}>"];
        }

        if (operation is IBinaryOperation
            {
                OperatorKind: BinaryOperatorKind.Add,
                Type.SpecialType: SpecialType.System_String,
            } binary)
        {
            string[] left =
                DescribeGlobalNamePatterns(binary.LeftOperand);
            string[] right =
                DescribeGlobalNamePatterns(binary.RightOperand);
            return (
                from leftPart in left
                from rightPart in right
                select leftPart + rightPart).ToArray();
        }

        return operation.Type?.SpecialType == SpecialType.System_String
            ? ["<unknown>"]
            : [];
    }

    private DescriptorProvenanceValue EvaluateArgument(
        IInvocationOperation invocation,
        int parameterOrdinal,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        IArgumentOperation? argument = invocation.Arguments.FirstOrDefault(
            candidate => candidate.Parameter?.Ordinal == parameterOrdinal);
        return argument is null
            ? DescriptorProvenanceValue.Bottom
            : Evaluate(argument.Value, state, sink);
    }

    private static FiniteSetValue<DescriptorName> DescriptorNamesFromStrings(FiniteSetValue<string> strings)
    {
        if (strings.IsUnknown)
            return FiniteSetValue<DescriptorName>.Unknown;

        FiniteSetValue<DescriptorName> value = FiniteSetValue<DescriptorName>.Bottom;
        foreach (string name in strings.Values)
            value = value.Join(FiniteSetValue<DescriptorName>.Known(new DescriptorName(name)));
        return value;
    }

    private DescriptorProvenanceValue EvaluateProperty(
        IPropertyReferenceOperation property,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        if (_apiSymbols.TypeInfoFields.Matches(property.Property))
        {
            return DescriptorProvenanceValue.FromFieldsOwner(
                Evaluate(property.Instance!, state, sink).TypeInfo);
        }

        if (_apiSymbols.TypeInfoSize.Matches(property.Property))
        {
            FiniteSetValue<DescriptorName> typeInfo = Evaluate(property.Instance!, state, sink).TypeInfo;
            RecordFields(typeInfo, FiniteSetValue<string>.Known("Size"), UsageKind.Read, sink);
            return DescriptorProvenanceValue.Bottom;
        }

        if (_apiSymbols.FieldInfoOffset.Matches(property.Property))
        {
            return DescriptorProvenanceValue.FromOffsets(
                Evaluate(property.Instance!, state, sink).Fields);
        }

        if (property.Property.IsIndexer)
        {
            DescriptorProvenanceValue instance = Evaluate(property.Instance!, state, sink);
            if (!instance.FieldsOwner.Equals(FiniteSetValue<DescriptorName>.Bottom))
            {
                FiniteSetValue<string> names = FiniteSetValue<string>.Bottom;
                foreach (IArgumentOperation argument in property.Arguments)
                    names = names.Join(Evaluate(argument.Value, state, sink).Strings);
                RecordFields(instance.FieldsOwner, names, UsageKind.OffsetLookup, sink);
                return DescriptorProvenanceValue.FromFields(CreateFields(instance.FieldsOwner, names));
            }
        }

        return EvaluateChildren(property, state, sink);
    }

    private DescriptorProvenanceValue EvaluateBinary(
        IBinaryOperation binary,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        DescriptorProvenanceValue left = Evaluate(binary.LeftOperand, state, sink);
        DescriptorProvenanceValue right = Evaluate(binary.RightOperand, state, sink);
        FiniteSetValue<FieldIdentity> fields = left.Addresses
            .Join(right.Addresses)
            .Join(left.Offsets)
            .Join(right.Offsets);
        return fields.Equals(FiniteSetValue<FieldIdentity>.Bottom)
            ? DescriptorProvenanceValue.Bottom
            : DescriptorProvenanceValue.FromAddresses(fields);
    }

    private DescriptorProvenanceValue EvaluateChildren(
        IOperation operation,
        FlowState<DescriptorProvenanceValue> state,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        foreach (IOperation child in operation.ChildOperations)
            Evaluate(child, state, sink);
        return DescriptorProvenanceValue.Bottom;
    }

    private static DescriptorProvenanceValue Assign(
        FlowSlot? target,
        DescriptorProvenanceValue value,
        FlowState<DescriptorProvenanceValue> state)
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

    private static void RecordAddressEffects(
        DescriptorProvenanceValue address,
        UsageKind usage,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        foreach (FieldIdentity field in address.Addresses.Values)
            sink.AddEffect(new FieldAccessEffect(field, usage));
    }

    private static void RecordFields(
        FiniteSetValue<DescriptorName> typeInfo,
        FiniteSetValue<string> names,
        UsageKind usage,
        IFlowResultSink<DescriptorProvenanceValue, CdacEffect> sink)
    {
        foreach (FieldIdentity field in CreateFields(typeInfo, names).Values)
            sink.AddEffect(new FieldAccessEffect(field, usage));
    }

    private static FiniteSetValue<FieldIdentity> CreateFields(
        FiniteSetValue<DescriptorName> typeInfo,
        FiniteSetValue<string> names)
    {
        if (typeInfo.IsUnknown || names.IsUnknown)
            return FiniteSetValue<FieldIdentity>.Unknown;

        return FiniteSetValue<FieldIdentity>.Known(
            from descriptor in typeInfo.Values
            from fieldName in names.Values
            select new FieldIdentity(descriptor, fieldName));
    }
}
