// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

internal readonly struct CodeNotifications_1 : ICodeNotifications
{
    private const uint CLRDATA_METHNOTIFY_NONE = 0;
    private const uint CLRDATA_METHNOTIFY_GENERATED = 1;
    private const uint CLRDATA_METHNOTIFY_DISCARDED = 2;

    private readonly Target _target;

    internal CodeNotifications_1(Target target)
    {
        _target = target;
    }

    void ICodeNotifications.SetCodeNotification(TargetPointer module, uint methodToken, uint flags)
    {
        if (!IsValidMethodCodeNotification(flags))
            throw new ArgumentException("Invalid code notification flags", nameof(flags));

        uint entrySize = GetEntrySize();
        TargetPointer tablePointer = ReadTablePointer();

        if (tablePointer == TargetPointer.Null && flags == CLRDATA_METHNOTIFY_NONE)
            return;

        if (tablePointer == TargetPointer.Null)
        {
            tablePointer = AllocateTable(entrySize);
        }

        // Bookkeeping is at index 0: methodToken field stores length.
        // Capacity is a compile-time invariant exposed via the JITNotificationTableSize global.
        Data.JITNotification bookkeeping = new(_target, tablePointer);
        uint length = bookkeeping.MethodToken;
        uint capacity = _target.ReadGlobal<uint>(Constants.Globals.JITNotificationTableSize);
        ulong entriesBase = tablePointer + entrySize;

        if (flags == CLRDATA_METHNOTIFY_NONE)
        {
            if (TryFindEntry(entriesBase, entrySize, length, module, methodToken, out uint foundIndex))
            {
                Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(foundIndex * entrySize)));
                entry.Clear(_target);
                if (foundIndex == length - 1)
                {
                    bookkeeping.WriteMethodToken(_target, length - 1);
                }
            }

            return;
        }

        if (TryFindEntry(entriesBase, entrySize, length, module, methodToken, out uint existingIndex))
        {
            Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(existingIndex * entrySize)));
            entry.WriteState(_target, (ushort)flags);

            return;
        }

        uint firstFree = length;
        for (uint i = 0; i < length; i++)
        {
            Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(i * entrySize)));
            if (entry.IsFree)
            {
                firstFree = i;
                break;
            }
        }

        if (firstFree >= capacity)
            throw new InvalidOperationException("JIT notification table is full");

        Data.JITNotification newEntry = new(_target, new TargetPointer(entriesBase + (ulong)(firstFree * entrySize)));
        newEntry.WriteEntry(_target, module, methodToken, (ushort)flags);

        if (firstFree >= length)
        {
            bookkeeping.WriteMethodToken(_target, length + 1);
        }
    }

    uint ICodeNotifications.GetCodeNotification(TargetPointer module, uint methodToken)
    {
        uint entrySize = GetEntrySize();
        TargetPointer tablePointer = ReadTablePointer();

        if (tablePointer == TargetPointer.Null)
            throw new InvalidOperationException("JIT notification table not allocated");

        Data.JITNotification bookkeeping = new(_target, tablePointer);
        uint length = bookkeeping.MethodToken;
        ulong entriesBase = tablePointer + entrySize;

        if (TryFindEntry(entriesBase, entrySize, length, module, methodToken, out uint foundIndex))
        {
            Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(foundIndex * entrySize)));

            return entry.State;
        }

        return CLRDATA_METHNOTIFY_NONE;
    }

    void ICodeNotifications.SetAllCodeNotifications(TargetPointer module, uint flags)
    {
        if (!IsValidMethodCodeNotification(flags))
            throw new ArgumentException("Invalid code notification flags", nameof(flags));

        uint entrySize = GetEntrySize();
        TargetPointer tablePointer = ReadTablePointer();

        if (tablePointer == TargetPointer.Null)
            return;

        Data.JITNotification bookkeeping = new(_target, tablePointer);
        uint length = bookkeeping.MethodToken;
        ulong entriesBase = tablePointer + entrySize;

        bool changed = false;
        for (uint i = 0; i < length; i++)
        {
            Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(i * entrySize)));
            if (entry.IsFree)
                continue;

            if (module != TargetPointer.Null)
            {
                if (entry.ClrModule.Value != module.Value)
                    continue;
            }

            if (flags == CLRDATA_METHNOTIFY_NONE)
            {
                entry.Clear(_target);
            }
            else
            {
                entry.WriteState(_target, (ushort)flags);
            }

            changed = true;
        }

        if (changed && flags == CLRDATA_METHNOTIFY_NONE)
        {
            uint newLength = length;
            while (newLength > 0)
            {
                Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)((newLength - 1) * entrySize)));
                if (!entry.IsFree)
                    break;
                newLength--;
            }

            bookkeeping.WriteMethodToken(_target, newLength);
        }
    }

    uint ICodeNotifications.GetCodeNotificationCapacity()
    {
        return _target.ReadGlobal<uint>(Constants.Globals.JITNotificationTableSize);
    }

    private bool TryFindEntry(
        ulong entriesBase, uint entrySize,
        uint length,
        TargetPointer module, uint methodToken,
        out uint index)
    {
        for (uint i = 0; i < length; i++)
        {
            Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(i * entrySize)));
            if (entry.IsFree)
                continue;

            if (entry.ClrModule.Value != module.Value)
                continue;

            if (entry.MethodToken != methodToken)
                continue;

            index = i;

            return true;
        }

        index = 0;

        return false;
    }

    private static bool IsValidMethodCodeNotification(uint flags)
    {
        return (flags & ~(CLRDATA_METHNOTIFY_GENERATED | CLRDATA_METHNOTIFY_DISCARDED)) == 0;
    }

    private uint GetEntrySize()
    {
        Target.TypeInfo jitNotifType = _target.GetTypeInfo(DataType.JITNotification);
        return (uint)jitNotifType.Size!.Value;
    }

    private TargetPointer ReadTablePointer()
    {
        return _target.ReadPointer(
            _target.ReadGlobalPointer(Constants.Globals.JITNotificationTable));
    }

    /// <summary>
    /// Lazily allocate a JIT notification table in the target process using AllocateMemory,
    /// zero-fill it (slot 0's methodToken is the length, which starts at 0), and write the
    /// pointer back to g_pNotificationTable.
    /// </summary>
    private TargetPointer AllocateTable(uint entrySize)
    {
        uint capacity = _target.ReadGlobal<uint>(Constants.Globals.JITNotificationTableSize);
        // Table has capacity+1 entries: index 0 is bookkeeping
        uint tableByteSize = entrySize * (capacity + 1);
        TargetPointer tablePointer = _target.AllocateMemory(tableByteSize);

        byte[] zeros = new byte[tableByteSize];
        _target.WriteBuffer(tablePointer.Value, zeros);

        TargetPointer globalAddr = _target.ReadGlobalPointer(Constants.Globals.JITNotificationTable);
        WriteNUInt(globalAddr, tablePointer);

        return tablePointer;
    }

    private void WriteNUInt(ulong address, TargetPointer value)
    {
        if (_target.PointerSize == 8)
            _target.Write<ulong>(address, value.Value);
        else
            _target.Write<uint>(address, (uint)value.Value);
    }
}
