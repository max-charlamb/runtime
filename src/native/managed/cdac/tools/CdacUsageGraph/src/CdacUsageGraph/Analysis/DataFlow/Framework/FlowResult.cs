// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// The outcome of a <see cref="ForwardControlFlowSolver{TValue, TEffect}"/> run over one method
/// or constructor body: the joined value observed at every visited operation, the effects
/// recorded by the transfer function, the joined return value, and the CFG exit state (used to
/// read <c>out</c>/<c>ref</c> parameter values after the call).
/// </summary>
internal sealed class FlowResult<TValue, TEffect>
    where TValue : IFlowValue<TValue>
{
    private readonly IReadOnlyDictionary<OperationKey, TValue> _values;

    public FlowResult(
        IReadOnlyDictionary<OperationKey, TValue> values,
        IReadOnlyCollection<TEffect> effects,
        TValue returnValue,
        FlowState<TValue> exitState)
    {
        _values = values;
        Effects = effects;
        ReturnValue = returnValue;
        ExitState = exitState;
    }

    public TValue GetValue(IOperation operation) =>
        _values.TryGetValue(OperationKey.Create(operation), out TValue? value)
            ? value
            : TValue.Bottom;

    public IReadOnlyCollection<TEffect> Effects { get; }

    public TValue ReturnValue { get; }

    public FlowState<TValue> ExitState { get; }
}
