// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>Possible cDAC names described by a TypeInfo value at one program point.</summary>
internal sealed class TypeInfoValue : IEquatable<TypeInfoValue>
{
    private readonly HashSet<string> _names;

    private TypeInfoValue(bool isUnknown, IEnumerable<string> names)
    {
        IsUnknown = isUnknown;
        _names = new HashSet<string>(names, StringComparer.Ordinal);
    }

    public static TypeInfoValue Bottom { get; } = new(false, []);

    public static TypeInfoValue Unknown { get; } = new(true, []);

    public bool IsUnknown { get; }

    public IReadOnlyCollection<string> Names => _names;

    public static TypeInfoValue Known(string name) => new(false, [name]);

    public TypeInfoValue Join(TypeInfoValue other)
    {
        if (IsUnknown || other.IsUnknown)
            return Unknown;
        if (_names.IsSupersetOf(other._names))
            return this;
        if (other._names.IsSupersetOf(_names))
            return other;

        return new TypeInfoValue(false, _names.Concat(other._names));
    }

    public bool Equals(TypeInfoValue? other) =>
        other is not null &&
        IsUnknown == other.IsUnknown &&
        _names.SetEquals(other._names);

    public override bool Equals(object? obj) => Equals(obj as TypeInfoValue);

    public override int GetHashCode()
    {
        int namesHash = 0;
        foreach (string name in _names)
            namesHash ^= StringComparer.Ordinal.GetHashCode(name);
        return HashCode.Combine(IsUnknown, namesHash);
    }
}
