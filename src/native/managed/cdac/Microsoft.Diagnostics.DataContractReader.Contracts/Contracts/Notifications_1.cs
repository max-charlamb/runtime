// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Diagnostics.CodeAnalysis;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

internal readonly struct Notifications_1 : INotifications
{
    private const uint CLRDATA_METHNOTIFY_NONE = 0;
    private const uint CLRDATA_METHNOTIFY_GENERATED = 1;
    private const uint CLRDATA_METHNOTIFY_DISCARDED = 2;

    private enum NotificationType_1 : uint
    {
        ModuleLoad = 1,
        ModuleUnload = 2,
        Exception = 5,
        Gc = 6,
        ExceptionCatcherEnter = 7,
        Jit2 = 8,
    }

    private readonly Target _target;

    internal Notifications_1(Target target)
    {
        _target = target;
    }

    void INotifications.SetGcNotification(int condemnedGeneration)
    {
        TargetPointer pGcNotificationFlags = _target.ReadGlobalPointer(Constants.Globals.GcNotificationFlags);
        uint currentFlags = _target.Read<uint>(pGcNotificationFlags);
        if (condemnedGeneration == 0)
            _target.Write<uint>(pGcNotificationFlags, 0);
        else
        {
            _target.Write<uint>(pGcNotificationFlags, currentFlags | (uint)condemnedGeneration);
        }
    }

    bool INotifications.TryParseNotification(ReadOnlySpan<TargetPointer> exceptionInformation, [NotNullWhen(true)] out NotificationData? notification)
    {
        notification = null;

        if (exceptionInformation.IsEmpty)
            return false;

        notification = (NotificationType_1)(uint)exceptionInformation[0].Value switch
        {
            NotificationType_1.ModuleLoad => new ModuleLoadNotificationData(exceptionInformation[1]),
            NotificationType_1.ModuleUnload => new ModuleUnloadNotificationData(exceptionInformation[1]),
            NotificationType_1.Jit2 => new JitNotificationData(exceptionInformation[1], exceptionInformation[2]),
            NotificationType_1.Exception => new ExceptionNotificationData(exceptionInformation[1]),
            NotificationType_1.Gc => ParseGcNotification(exceptionInformation),
            NotificationType_1.ExceptionCatcherEnter => new ExceptionCatcherEnterNotificationData(exceptionInformation[1], (uint)exceptionInformation[2].Value),
            _ => null,
        };

        return notification is not null;
    }

    private static GcNotificationData ParseGcNotification(ReadOnlySpan<TargetPointer> exceptionInformation)
    {
        GcEventType eventType = (GcEventType)(uint)exceptionInformation[1].Value;
        GcEventData eventData = new(eventType, (int)(uint)exceptionInformation[2].Value);
        return new GcNotificationData(eventData, IsSupportedEvent: eventType == GcEventType.MarkEnd);
    }

    void INotifications.SetCodeNotification(TargetPointer module, uint methodToken, uint flags)
    {
        if (!IsValidMethodCodeNotification(flags))
            throw new ArgumentException("Invalid code notification flags", nameof(flags));

        uint entrySize = GetEntrySize();
        TargetPointer tablePointer = ReadTablePointer();

        // If table is null and we're clearing, nothing to do
        if (tablePointer == TargetPointer.Null && flags == CLRDATA_METHNOTIFY_NONE)
            return;

        // If table is null and we're setting, lazily allocate
        if (tablePointer == TargetPointer.Null)
        {
            tablePointer = AllocateTable(entrySize);
        }

        // Bookkeeping is at index 0: methodToken field stores length, clrModule field stores capacity
        Data.JITNotification bookkeeping = new(_target, tablePointer);
        uint length = bookkeeping.MethodToken;
        uint capacity = (uint)bookkeeping.ClrModule.Value;
        ulong entriesBase = tablePointer + entrySize;

        if (flags == CLRDATA_METHNOTIFY_NONE)
        {
            // Remove: find and clear the entry
            if (TryFindEntry(entriesBase, entrySize, length, module, methodToken, out uint foundIndex))
            {
                Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(foundIndex * entrySize)));
                entry.Clear(_target);
                // If this was the last entry, decrement length
                if (foundIndex == length - 1)
                {
                    bookkeeping.WriteMethodToken(_target, length - 1);
                }
            }

            return;
        }

        // Update existing entry
        if (TryFindEntry(entriesBase, entrySize, length, module, methodToken, out uint existingIndex))
        {
            Data.JITNotification entry = new(_target, new TargetPointer(entriesBase + (ulong)(existingIndex * entrySize)));
            entry.WriteState(_target, (ushort)flags);

            return;
        }

        // Find first free slot
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

        // Write new entry
        Data.JITNotification newEntry = new(_target, new TargetPointer(entriesBase + (ulong)(firstFree * entrySize)));
        newEntry.WriteEntry(_target, module, methodToken, (ushort)flags);

        // Update length if we used a slot at the end
        if (firstFree >= length)
        {
            bookkeeping.WriteMethodToken(_target, length + 1);
        }
    }

    uint INotifications.GetCodeNotification(TargetPointer module, uint methodToken)
    {
        uint entrySize = GetEntrySize();
        TargetPointer tablePointer = ReadTablePointer();

        if (tablePointer == TargetPointer.Null)
            return CLRDATA_METHNOTIFY_NONE;

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

    void INotifications.SetAllCodeNotifications(TargetPointer module, uint flags)
    {
        if (!IsValidMethodCodeNotification(flags))
            throw new ArgumentException("Invalid code notification flags", nameof(flags));

        uint entrySize = GetEntrySize();
        TargetPointer tablePointer = ReadTablePointer();

        // If table is null, nothing to set/clear on
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

            // If a module filter is specified, check it
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

        // Update length if we cleared entries at the end
        if (changed && flags == CLRDATA_METHNOTIFY_NONE)
        {
            // Recalculate length: find the last non-free entry
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
    /// initialize its bookkeeping entry, and write the pointer back to g_pNotificationTable.
    /// </summary>
    private TargetPointer AllocateTable(uint entrySize)
    {
        uint capacity = _target.ReadGlobal<uint>(Constants.Globals.JITNotificationTableSize);
        // Table has capacity+1 entries: index 0 is bookkeeping
        uint tableByteSize = entrySize * (capacity + 1);
        TargetPointer tablePointer = _target.AllocateMemory(tableByteSize);

        // Zero-initialize the entire table
        byte[] zeros = new byte[tableByteSize];
        _target.WriteBuffer(tablePointer.Value, zeros);

        // Initialize bookkeeping at index 0 via the Data class
        Data.JITNotification bookkeeping = new(_target, tablePointer);
        bookkeeping.WriteMethodToken(_target, 0); // length = 0
        bookkeeping.WriteClrModule(_target, new TargetPointer(capacity)); // capacity

        // Write the table pointer back to the global
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
