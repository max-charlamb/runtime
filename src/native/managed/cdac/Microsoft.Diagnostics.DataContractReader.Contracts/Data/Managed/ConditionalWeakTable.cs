// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader.Data;

internal sealed class ConditionalWeakTable : IData<ConditionalWeakTable>
{
    public const string Namespace = "System.Runtime.CompilerServices";
    public const string Name = "ConditionalWeakTable`2";

    static ConditionalWeakTable IData<ConditionalWeakTable>.Create(Target target, TargetPointer address)
        => new ConditionalWeakTable(target, address);

    public ConditionalWeakTable(Target target, TargetPointer address)
    {
        ManagedTypeInfo type = target.Contracts.ManagedTypeLayout.GetTypeInfo(Namespace, Name);

        Container = target.ReadPointerField(address, type, "_container");
    }

    public TargetPointer Container { get; init; }
}
