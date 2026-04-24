// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader.Data;

internal sealed class NativeObjectWrapperObject : IData<NativeObjectWrapperObject>
{
    public const string Namespace = "System.Runtime.InteropServices";
    public const string Name = "ComWrappers+NativeObjectWrapper";

    static NativeObjectWrapperObject IData<NativeObjectWrapperObject>.Create(Target target, TargetPointer address) => new NativeObjectWrapperObject(target, address);

    public NativeObjectWrapperObject(Target target, TargetPointer address)
    {
        ManagedTypeInfo type = target.Contracts.ManagedTypeLayout.GetTypeInfo(Namespace, Name);

        ExternalComObject = target.ReadPointerField(address, type, "_externalComObject");
    }

    public TargetPointer ExternalComObject { get; init; }
}
