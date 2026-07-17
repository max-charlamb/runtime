// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph;

/// <summary>
/// Stable cDAC project, metadata, and member names used as semantic anchors by the analysis.
/// Keeping these names together makes source-layout or API drift visible and avoids treating
/// repeated string literals as independent facts.
/// </summary>
internal static class CdacSymbols
{
    public const string AbstractionsProjectName = "Microsoft.Diagnostics.DataContractReader.Abstractions";
    public const string ContractsProjectDirectory = "Microsoft.Diagnostics.DataContractReader.Contracts";
    public const string ContractsProjectFile = "Microsoft.Diagnostics.DataContractReader.Contracts.csproj";

    public const string ContractRegistryMetadataName =
        "Microsoft.Diagnostics.DataContractReader.ContractRegistry";
    public const string TargetMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Target";
    public const string TargetTypeInfoMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Target+TypeInfo";
    public const string TargetFieldInfoMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Target+FieldInfo";
    public const string TargetDataCacheMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Target+IDataCache";
    public const string IContractMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Contracts.IContract";
    public const string IManagedTypeSourceMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Contracts.IManagedTypeSource";
    public const string DataTypeNamesMetadataName =
        "Microsoft.Diagnostics.DataContractReader.DataTypeNames";
    public const string CoreCLRContractsMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Contracts.CoreCLRContracts";
    public const string IDataMetadataName =
        "Microsoft.Diagnostics.DataContractReader.Data.IData`1";

    public const string CdacTypeAttributeMetadataName =
        "Microsoft.Diagnostics.DataContractReader.CdacTypeAttribute";
    public const string FieldAttributeMetadataName =
        "Microsoft.Diagnostics.DataContractReader.FieldAttribute";
    public const string FieldAddressAttributeMetadataName =
        "Microsoft.Diagnostics.DataContractReader.FieldAddressAttribute";
    public const string RawOffsetAttributeMetadataName =
        "Microsoft.Diagnostics.DataContractReader.RawOffsetAttribute";
    public const string InstanceDataStartAttributeMetadataName =
        "Microsoft.Diagnostics.DataContractReader.InstanceDataStartAttribute";
    public const string ContractRegistrationMethodName = "Register";
    public const string DataInitializerMethodName = "OnInit";
    public const string TargetTypeName = "Target";
    public const string TypeInfoTypeName = "TypeInfo";
    public const string GetTypeInfoMethodName = "GetTypeInfo";
    public const string TryGetTypeInfoMethodName = "TryGetTypeInfo";
    public const string ReadGlobalMethodName = "ReadGlobal";
    public const string TryReadGlobalMethodName = "TryReadGlobal";
    public const string ReadGlobalPointerMethodName = "ReadGlobalPointer";
    public const string TryReadGlobalPointerMethodName = "TryReadGlobalPointer";
    public const string ReadGlobalStringMethodName = "ReadGlobalString";
    public const string TryReadGlobalStringMethodName = "TryReadGlobalString";
    public const string ToNameMethodName = "ToName";
}
