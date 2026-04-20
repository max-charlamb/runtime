// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace Microsoft.Diagnostics.DataContractReader.Data;

internal sealed class JITNotification : IData<JITNotification>
{
    static JITNotification IData<JITNotification>.Create(Target target, TargetPointer address)
        => new JITNotification(target, address);

    private readonly TargetPointer _address;

    public JITNotification(Target target, TargetPointer address)
    {
        _address = address;
        Target.TypeInfo type = target.GetTypeInfo(DataType.JITNotification);

        State = target.ReadField<ushort>(address, type, nameof(State));
        ClrModule = target.ReadNUIntField(address, type, nameof(ClrModule));
        MethodToken = target.ReadField<uint>(address, type, nameof(MethodToken));
    }

    public ushort State { get; private set; }
    public TargetNUInt ClrModule { get; private set; }
    public uint MethodToken { get; private set; }

    public bool IsFree => State == 0;

    public void WriteState(Target target, ushort state)
    {
        Target.TypeInfo type = target.GetTypeInfo(DataType.JITNotification);
        ulong addr = _address + (ulong)type.Fields[nameof(State)].Offset;
        target.Write<ushort>(addr, state);
        State = state;
    }

    public void WriteClrModule(Target target, TargetPointer module)
    {
        Target.TypeInfo type = target.GetTypeInfo(DataType.JITNotification);
        ulong addr = _address + (ulong)type.Fields[nameof(ClrModule)].Offset;
        if (target.PointerSize == 8)
            target.Write<ulong>(addr, module.Value);
        else
            target.Write<uint>(addr, (uint)module.Value);
        ClrModule = new TargetNUInt(module.Value);
    }

    public void WriteMethodToken(Target target, uint methodToken)
    {
        Target.TypeInfo type = target.GetTypeInfo(DataType.JITNotification);
        ulong addr = _address + (ulong)type.Fields[nameof(MethodToken)].Offset;
        target.Write<uint>(addr, methodToken);
        MethodToken = methodToken;
    }

    public void Clear(Target target)
    {
        WriteState(target, 0);
        WriteClrModule(target, TargetPointer.Null);
        WriteMethodToken(target, 0);
    }

    public void WriteEntry(Target target, TargetPointer module, uint methodToken, ushort state)
    {
        WriteClrModule(target, module);
        WriteMethodToken(target, methodToken);
        WriteState(target, state);
    }
}
