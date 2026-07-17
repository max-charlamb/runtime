// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

internal sealed class StringValue : IEquatable<StringValue>
{
    private readonly HashSet<string> _values;

    private StringValue(bool isUnknown, IEnumerable<string> values)
    {
        IsUnknown = isUnknown;
        _values = new HashSet<string>(values, StringComparer.Ordinal);
    }

    public static StringValue Bottom { get; } = new(false, []);
    public static StringValue Unknown { get; } = new(true, []);

    public bool IsUnknown { get; }
    public IReadOnlyCollection<string> Values => _values;

    public static StringValue Known(string value) => new(false, [value]);

    public StringValue Join(StringValue other)
    {
        if (IsUnknown || other.IsUnknown)
            return Unknown;
        if (_values.IsSupersetOf(other._values))
            return this;
        if (other._values.IsSupersetOf(_values))
            return other;
        return new StringValue(false, _values.Concat(other._values));
    }

    public bool Equals(StringValue? other) =>
        other is not null && IsUnknown == other.IsUnknown && _values.SetEquals(other._values);

    public override bool Equals(object? obj) => Equals(obj as StringValue);

    public override int GetHashCode()
    {
        int valuesHash = 0;
        foreach (string value in _values)
            valuesHash ^= StringComparer.Ordinal.GetHashCode(value);
        return HashCode.Combine(IsUnknown, valuesHash);
    }
}
