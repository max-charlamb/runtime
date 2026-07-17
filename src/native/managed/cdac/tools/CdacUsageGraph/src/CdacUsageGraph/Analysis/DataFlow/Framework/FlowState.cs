// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// A generic per-program-point dataflow state: a sparse map from <see cref="FlowSlot"/> (locals,
/// parameters, fields, and flow captures) to a lattice value, with bottom values not stored.
/// </summary>
internal sealed class FlowState<TValue> : IEquatable<FlowState<TValue>>
    where TValue : IFlowValue<TValue>
{
    private readonly Dictionary<FlowSlot, TValue> _values;

    public FlowState() =>
        _values = new Dictionary<FlowSlot, TValue>(FlowSlotComparer.Instance);

    private FlowState(Dictionary<FlowSlot, TValue> values) => _values = values;

    public TValue this[FlowSlot slot]
    {
        get => _values.TryGetValue(slot, out TValue? value) ? value : TValue.Bottom;
        set
        {
            if (value.Equals(TValue.Bottom))
                _values.Remove(slot);
            else
                _values[slot] = value;
        }
    }

    public FlowState<TValue> Clone() =>
        new(new Dictionary<FlowSlot, TValue>(_values, FlowSlotComparer.Instance));

    public bool JoinFrom(FlowState<TValue> other)
    {
        bool changed = false;
        foreach (KeyValuePair<FlowSlot, TValue> entry in other._values)
        {
            TValue joined = this[entry.Key].Join(entry.Value);
            if (!joined.Equals(this[entry.Key]))
            {
                this[entry.Key] = joined;
                changed = true;
            }
        }
        return changed;
    }

    public bool Equals(FlowState<TValue>? other)
    {
        if (other is null || _values.Count != other._values.Count)
            return false;
        foreach (KeyValuePair<FlowSlot, TValue> entry in _values)
        {
            if (!entry.Value.Equals(other[entry.Key]))
                return false;
        }
        return true;
    }

    public override bool Equals(object? obj) => Equals(obj as FlowState<TValue>);

    public override int GetHashCode() => _values.Count;
}
