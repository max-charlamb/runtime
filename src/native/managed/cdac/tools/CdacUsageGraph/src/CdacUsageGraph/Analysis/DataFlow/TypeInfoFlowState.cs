// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

internal sealed class TypeInfoFlowState : IEquatable<TypeInfoFlowState>
{
    private readonly Dictionary<FlowSlot, ProvenanceValue> _values;

    public TypeInfoFlowState() =>
        _values = new Dictionary<FlowSlot, ProvenanceValue>(FlowSlotComparer.Instance);

    private TypeInfoFlowState(Dictionary<FlowSlot, ProvenanceValue> values) => _values = values;

    public ProvenanceValue this[FlowSlot slot]
    {
        get => _values.TryGetValue(slot, out ProvenanceValue? value) ? value : ProvenanceValue.Bottom;
        set
        {
            if (value.Equals(ProvenanceValue.Bottom))
                _values.Remove(slot);
            else
                _values[slot] = value;
        }
    }

    public TypeInfoFlowState Clone() =>
        new(new Dictionary<FlowSlot, ProvenanceValue>(_values, FlowSlotComparer.Instance));

    public bool JoinFrom(TypeInfoFlowState other)
    {
        bool changed = false;
        foreach (KeyValuePair<FlowSlot, ProvenanceValue> entry in other._values)
        {
            ProvenanceValue joined = this[entry.Key].Join(entry.Value);
            if (!joined.Equals(this[entry.Key]))
            {
                this[entry.Key] = joined;
                changed = true;
            }
        }
        return changed;
    }

    public bool Equals(TypeInfoFlowState? other)
    {
        if (other is null || _values.Count != other._values.Count)
            return false;
        foreach (KeyValuePair<FlowSlot, ProvenanceValue> entry in _values)
        {
            if (!entry.Value.Equals(other[entry.Key]))
                return false;
        }
        return true;
    }

    public override bool Equals(object? obj) => Equals(obj as TypeInfoFlowState);

    public override int GetHashCode() => _values.Count;
}
