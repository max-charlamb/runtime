// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis.DataFlow.Framework;

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// The cDAC provenance lattice value tracked at every program point: the possible descriptor
/// (`Target.TypeInfo`) names, string literals/constants, `TypeInfo.Fields` owners, and resolved
/// field/offset/address identities a value may carry. Each component is an independent
/// <see cref="FiniteSetValue{T}"/> lattice; joining two provenance values joins each component.
/// </summary>
internal sealed class DescriptorProvenanceValue :
    IEquatable<DescriptorProvenanceValue>,
    IFlowValue<DescriptorProvenanceValue>
{
    public static DescriptorProvenanceValue Bottom { get; } = new(
        FiniteSetValue<DescriptorName>.Bottom,
        FiniteSetValue<string>.Bottom,
        FiniteSetValue<DescriptorName>.Bottom,
        FiniteSetValue<FieldIdentity>.Bottom,
        FiniteSetValue<FieldIdentity>.Bottom,
        FiniteSetValue<FieldIdentity>.Bottom);

    public DescriptorProvenanceValue(
        FiniteSetValue<DescriptorName> typeInfo,
        FiniteSetValue<string> strings,
        FiniteSetValue<DescriptorName> fieldsOwner,
        FiniteSetValue<FieldIdentity> fields,
        FiniteSetValue<FieldIdentity> offsets,
        FiniteSetValue<FieldIdentity> addresses)
    {
        TypeInfo = typeInfo;
        Strings = strings;
        FieldsOwner = fieldsOwner;
        Fields = fields;
        Offsets = offsets;
        Addresses = addresses;
    }

    public FiniteSetValue<DescriptorName> TypeInfo { get; }
    public FiniteSetValue<string> Strings { get; }
    public FiniteSetValue<DescriptorName> FieldsOwner { get; }
    public FiniteSetValue<FieldIdentity> Fields { get; }
    public FiniteSetValue<FieldIdentity> Offsets { get; }
    public FiniteSetValue<FieldIdentity> Addresses { get; }

    public static DescriptorProvenanceValue FromTypeInfo(FiniteSetValue<DescriptorName> value) =>
        new(
            value,
            FiniteSetValue<string>.Bottom,
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom);

    public static DescriptorProvenanceValue FromString(FiniteSetValue<string> value) =>
        new(
            FiniteSetValue<DescriptorName>.Bottom,
            value,
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom);

    public static DescriptorProvenanceValue FromFieldsOwner(FiniteSetValue<DescriptorName> value) =>
        new(
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<string>.Bottom,
            value,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom);

    public static DescriptorProvenanceValue FromFields(FiniteSetValue<FieldIdentity> value) =>
        new(
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<string>.Bottom,
            FiniteSetValue<DescriptorName>.Bottom,
            value,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom);

    public static DescriptorProvenanceValue FromOffsets(FiniteSetValue<FieldIdentity> value) =>
        new(
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<string>.Bottom,
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            value,
            FiniteSetValue<FieldIdentity>.Bottom);

    public static DescriptorProvenanceValue FromAddresses(FiniteSetValue<FieldIdentity> value) =>
        new(
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<string>.Bottom,
            FiniteSetValue<DescriptorName>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            FiniteSetValue<FieldIdentity>.Bottom,
            value);

    public DescriptorProvenanceValue Join(DescriptorProvenanceValue other) => new(
        TypeInfo.Join(other.TypeInfo),
        Strings.Join(other.Strings),
        FieldsOwner.Join(other.FieldsOwner),
        Fields.Join(other.Fields),
        Offsets.Join(other.Offsets),
        Addresses.Join(other.Addresses));

    public bool Equals(DescriptorProvenanceValue? other) =>
        other is not null &&
        TypeInfo.Equals(other.TypeInfo) &&
        Strings.Equals(other.Strings) &&
        FieldsOwner.Equals(other.FieldsOwner) &&
        Fields.Equals(other.Fields) &&
        Offsets.Equals(other.Offsets) &&
        Addresses.Equals(other.Addresses);

    public override bool Equals(object? obj) => Equals(obj as DescriptorProvenanceValue);

    public override int GetHashCode() => HashCode.Combine(
        TypeInfo, Strings, FieldsOwner, Fields, Offsets, Addresses);
}
