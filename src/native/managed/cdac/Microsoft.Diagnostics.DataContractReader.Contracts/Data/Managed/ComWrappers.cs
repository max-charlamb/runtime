// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader.Data;

/// <summary>
/// Values of the <c>System.Runtime.InteropServices.ComWrappers</c> static fields that the cDAC
/// consumes. Not an <see cref="IData{T}"/> because static fields have no instance address.
/// </summary>
internal sealed class ComWrappers
{
    public const string Namespace = "System.Runtime.InteropServices";
    public const string Name = "ComWrappers";

    public ComWrappers(Target target)
    {
        ManagedTypeInfo type = target.Contracts.ManagedTypeLayout.GetTypeInfo(Namespace, Name);

        NativeObjectWrapperTable = target.ReadPointer(type.StaticFields["s_nativeObjectWrapperTable"]);
        AllManagedObjectWrapperTable = target.ReadPointer(type.StaticFields["s_allManagedObjectWrapperTable"]);
    }

    public TargetPointer NativeObjectWrapperTable { get; }
    public TargetPointer AllManagedObjectWrapperTable { get; }
}
