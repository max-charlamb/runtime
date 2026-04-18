// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace Microsoft.Diagnostics.DataContractReader.Data;

internal sealed class JITNotification : IData<JITNotification>
{
    static JITNotification IData<JITNotification>.Create(Target target, TargetPointer address)
        => new JITNotification(target, address);

    public JITNotification(Target target, TargetPointer address)
    {
        Target.TypeInfo type = target.GetTypeInfo(DataType.JITNotification);

        State = target.ReadField<ushort>(address, type, nameof(State));
        ClrModule = target.ReadNUIntField(address, type, nameof(ClrModule));
        MethodToken = target.ReadField<uint>(address, type, nameof(MethodToken));
    }

    public ushort State { get; init; }
    public TargetNUInt ClrModule { get; init; }
    public uint MethodToken { get; init; }
}
