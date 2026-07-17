// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// Sink passed to <see cref="IBlockTransfer{TValue, TEffect}.Evaluate"/> so a transfer function can
/// record the value observed at any (sub-)operation it visits and any effect it discovers, while
/// the <see cref="ForwardControlFlowSolver{TValue, TEffect}"/> that owns the sink remains ignorant
/// of what an operation value or an effect means. Repeated records for the same operation (e.g.
/// across worklist iterations) are joined rather than overwritten.
/// </summary>
internal interface IFlowResultSink<TValue, TEffect>
    where TValue : IFlowValue<TValue>
{
    void Record(IOperation operation, TValue value);

    void AddEffect(TEffect effect);

    void AddEffects(IEnumerable<TEffect> effects);
}
