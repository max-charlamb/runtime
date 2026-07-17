// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow;

internal sealed class TypeInfoFlowResult
{
    private readonly IReadOnlyDictionary<OperationKey, ProvenanceValue> _values;

    public TypeInfoFlowResult(
        IReadOnlyDictionary<OperationKey, ProvenanceValue> values,
        IReadOnlyCollection<FieldAccessEffect> effects,
        ProvenanceValue returnValue,
        TypeInfoFlowState exitState)
    {
        _values = values;
        Effects = effects;
        ReturnValue = returnValue;
        ExitState = exitState;
    }

    public TypeInfoValue GetValue(IOperation operation) =>
        GetProvenance(operation).TypeInfo;

    public ProvenanceValue GetProvenance(IOperation operation) =>
        _values.TryGetValue(OperationKey.Create(operation), out ProvenanceValue? value)
            ? value
            : ProvenanceValue.Bottom;

    public IReadOnlyCollection<FieldAccessEffect> Effects { get; }

    public ProvenanceValue ReturnValue { get; }

    public TypeInfoFlowState ExitState { get; }
}
