// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Collections.Immutable;

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// An immutable finite-set lattice value: bottom is the empty set, join is set union, and
/// <see cref="Unknown"/> is the top element absorbing any other value. String element sets use
/// ordinal comparison so provenance names never fold under culture-sensitive equality.
/// </summary>
internal sealed class FiniteSetValue<T> : IEquatable<FiniteSetValue<T>>, IFlowValue<FiniteSetValue<T>>
{
    private static readonly EqualityComparer<T> s_comparer = EqualityComparer<T>.Default;

    private readonly ImmutableHashSet<T> _values;
    private readonly int _hashCode;

    private FiniteSetValue(bool isUnknown, IEnumerable<T> values)
    {
        IsUnknown = isUnknown;
        _values = values.ToImmutableHashSet(s_comparer);

        int valuesHash = 0;
        foreach (T value in _values)
            valuesHash ^= s_comparer.GetHashCode(value!);
        _hashCode = HashCode.Combine(IsUnknown, _values.Count, valuesHash);
    }

    public static FiniteSetValue<T> Bottom { get; } = new(false, []);

    public static FiniteSetValue<T> Unknown { get; } = new(true, []);

    public bool IsUnknown { get; }

    public IReadOnlySet<T> Values => _values;

    public static FiniteSetValue<T> Known(T value) => new(false, [value]);

    public static FiniteSetValue<T> Known(IEnumerable<T> values) => new(false, values);

    public FiniteSetValue<T> Join(FiniteSetValue<T> other)
    {
        if (IsUnknown || other.IsUnknown)
            return Unknown;
        if (_values.IsSupersetOf(other._values))
            return this;
        if (other._values.IsSupersetOf(_values))
            return other;

        return new FiniteSetValue<T>(false, _values.Concat(other._values));
    }

    public bool Equals(FiniteSetValue<T>? other) =>
        ReferenceEquals(this, other) ||
        (other is not null &&
         _hashCode == other._hashCode &&
         IsUnknown == other.IsUnknown &&
         _values.SetEquals(other._values));

    public override bool Equals(object? obj) => Equals(obj as FiniteSetValue<T>);

    public override int GetHashCode() => _hashCode;
}
