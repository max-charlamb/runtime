// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

internal readonly record struct FieldIdentity(string TypeName, string FieldName);

internal sealed class FieldValue : IEquatable<FieldValue>
{
    private readonly HashSet<FieldIdentity> _fields;

    private FieldValue(bool isUnknown, IEnumerable<FieldIdentity> fields)
    {
        IsUnknown = isUnknown;
        _fields = new HashSet<FieldIdentity>(fields);
    }

    public static FieldValue Bottom { get; } = new(false, []);
    public static FieldValue Unknown { get; } = new(true, []);

    public bool IsUnknown { get; }
    public IReadOnlyCollection<FieldIdentity> Fields => _fields;

    public static FieldValue Known(IEnumerable<FieldIdentity> fields) => new(false, fields);

    public FieldValue Join(FieldValue other)
    {
        if (IsUnknown || other.IsUnknown)
            return Unknown;
        if (_fields.IsSupersetOf(other._fields))
            return this;
        if (other._fields.IsSupersetOf(_fields))
            return other;
        return new FieldValue(false, _fields.Concat(other._fields));
    }

    public bool Equals(FieldValue? other) =>
        other is not null && IsUnknown == other.IsUnknown && _fields.SetEquals(other._fields);

    public override bool Equals(object? obj) => Equals(obj as FieldValue);

    public override int GetHashCode()
    {
        int fieldsHash = 0;
        foreach (FieldIdentity field in _fields)
            fieldsHash ^= field.GetHashCode();
        return HashCode.Combine(IsUnknown, fieldsHash);
    }
}
