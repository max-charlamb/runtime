// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

internal sealed class ProvenanceValue : IEquatable<ProvenanceValue>
{
    public static ProvenanceValue Bottom { get; } = new(
        TypeInfoValue.Bottom,
        StringValue.Bottom,
        TypeInfoValue.Bottom,
        FieldValue.Bottom,
        FieldValue.Bottom,
        FieldValue.Bottom);

    public ProvenanceValue(
        TypeInfoValue typeInfo,
        StringValue strings,
        TypeInfoValue fieldsOwner,
        FieldValue fields,
        FieldValue offsets,
        FieldValue addresses)
    {
        TypeInfo = typeInfo;
        Strings = strings;
        FieldsOwner = fieldsOwner;
        Fields = fields;
        Offsets = offsets;
        Addresses = addresses;
    }

    public TypeInfoValue TypeInfo { get; }
    public StringValue Strings { get; }
    public TypeInfoValue FieldsOwner { get; }
    public FieldValue Fields { get; }
    public FieldValue Offsets { get; }
    public FieldValue Addresses { get; }

    public static ProvenanceValue FromTypeInfo(TypeInfoValue value) =>
        new(value, StringValue.Bottom, TypeInfoValue.Bottom, FieldValue.Bottom, FieldValue.Bottom, FieldValue.Bottom);

    public static ProvenanceValue FromString(StringValue value) =>
        new(TypeInfoValue.Bottom, value, TypeInfoValue.Bottom, FieldValue.Bottom, FieldValue.Bottom, FieldValue.Bottom);

    public static ProvenanceValue FromFieldsOwner(TypeInfoValue value) =>
        new(TypeInfoValue.Bottom, StringValue.Bottom, value, FieldValue.Bottom, FieldValue.Bottom, FieldValue.Bottom);

    public static ProvenanceValue FromFields(FieldValue value) =>
        new(TypeInfoValue.Bottom, StringValue.Bottom, TypeInfoValue.Bottom, value, FieldValue.Bottom, FieldValue.Bottom);

    public static ProvenanceValue FromOffsets(FieldValue value) =>
        new(TypeInfoValue.Bottom, StringValue.Bottom, TypeInfoValue.Bottom, FieldValue.Bottom, value, FieldValue.Bottom);

    public static ProvenanceValue FromAddresses(FieldValue value) =>
        new(TypeInfoValue.Bottom, StringValue.Bottom, TypeInfoValue.Bottom, FieldValue.Bottom, FieldValue.Bottom, value);

    public ProvenanceValue Join(ProvenanceValue other) => new(
        TypeInfo.Join(other.TypeInfo),
        Strings.Join(other.Strings),
        FieldsOwner.Join(other.FieldsOwner),
        Fields.Join(other.Fields),
        Offsets.Join(other.Offsets),
        Addresses.Join(other.Addresses));

    public bool Equals(ProvenanceValue? other) =>
        other is not null &&
        TypeInfo.Equals(other.TypeInfo) &&
        Strings.Equals(other.Strings) &&
        FieldsOwner.Equals(other.FieldsOwner) &&
        Fields.Equals(other.Fields) &&
        Offsets.Equals(other.Offsets) &&
        Addresses.Equals(other.Addresses);

    public override bool Equals(object? obj) => Equals(obj as ProvenanceValue);

    public override int GetHashCode() => HashCode.Combine(
        TypeInfo, Strings, FieldsOwner, Fields, Offsets, Addresses);
}
