// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// The per-analysis transfer function: evaluates one operation (and, recursively, its operands)
/// against the current <see cref="FlowState{TValue}"/>, returning the operation's value and
/// recording any per-operation values / effects it discovers through <paramref name="sink"/>.
/// Kept generic over <typeparamref name="TValue"/>/<typeparamref name="TEffect"/> so
/// <see cref="ForwardControlFlowSolver{TValue, TEffect}"/> stays free of analysis-specific
/// semantics (e.g. what an invocation of a particular API means).
/// </summary>
internal interface IOperationTransfer<TValue, TEffect>
    where TValue : IFlowValue<TValue>
{
    TValue Evaluate(IOperation operation, FlowState<TValue> state, IFlowResultSink<TValue, TEffect> sink);
}
