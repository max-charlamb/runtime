// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// Default <see cref="IFlowResultSink{TValue, TEffect}"/>: values recorded for the same operation
/// (e.g. because a block is re-evaluated when its input state changes, or a shared syntax node is
/// visited from more than one path) are joined; effects accumulate in a set across every
/// evaluation, including intermediate worklist iterations.
/// </summary>
internal sealed class FlowResultSink<TValue, TEffect> : IFlowResultSink<TValue, TEffect>
    where TValue : IFlowValue<TValue>
{
    private readonly Dictionary<OperationKey, TValue> _values = [];
    private readonly HashSet<TEffect> _effects = [];

    public IReadOnlyDictionary<OperationKey, TValue> Values => _values;

    public IReadOnlyCollection<TEffect> Effects => _effects;

    public void Record(IOperation operation, TValue value)
    {
        OperationKey key = OperationKey.Create(operation);
        _values[key] = _values.TryGetValue(key, out TValue? current) ? current.Join(value) : value;
    }

    public void AddEffect(TEffect effect) => _effects.Add(effect);

    public void AddEffects(IEnumerable<TEffect> effects) => _effects.UnionWith(effects);
}
