// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Diagnostics.CodeAnalysis;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

internal readonly struct Notifications_1 : INotifications
{
    private const ushort CLRDATA_METHNOTIFY_NONE = 0;
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

        Target.TypeInfo jitNotifType = _target.GetTypeInfo(DataType.JITNotification);
        int stateOffset = jitNotifType.Fields[nameof(Data.JITNotification.State)].Offset;
        int clrModuleOffset = jitNotifType.Fields[nameof(Data.JITNotification.ClrModule)].Offset;
        int methodTokenOffset = jitNotifType.Fields[nameof(Data.JITNotification.MethodToken)].Offset;
        uint entrySize = (uint)jitNotifType.Size!.Value;

        TargetPointer tablePointer = _target.ReadPointer(
            _target.ReadGlobalPointer(Constants.Globals.JITNotificationTable));

        if (tablePointer == TargetPointer.Null)
            throw new InvalidOperationException("JIT notification table is not initialized");

        // Bookkeeping is at index 0: methodToken field stores length, clrModule field stores capacity
        ulong bookkeepingAddr = tablePointer;
        uint length = _target.Read<uint>(bookkeepingAddr + (ulong)methodTokenOffset);
        uint capacity = (uint)_target.ReadNUInt(bookkeepingAddr + (ulong)clrModuleOffset).Value;
        ulong entriesBase = tablePointer + entrySize;

        if (flags == CLRDATA_METHNOTIFY_NONE)
        {
            // Remove: find and clear the entry
            if (TryFindEntry(entriesBase, entrySize, stateOffset, clrModuleOffset, methodTokenOffset, length, module, methodToken, out uint foundIndex))
            {
                ClearEntry(entriesBase, entrySize, stateOffset, clrModuleOffset, methodTokenOffset, foundIndex);
                // If this was the last entry, decrement length
                if (foundIndex == length - 1)
                {
                    _target.Write<uint>(bookkeepingAddr + (ulong)methodTokenOffset, length - 1);
                }
            }

            return;
        }

        // Update existing entry
        if (TryFindEntry(entriesBase, entrySize, stateOffset, clrModuleOffset, methodTokenOffset, length, module, methodToken, out uint existingIndex))
        {
            ulong entryAddr = entriesBase + (ulong)(existingIndex * entrySize);
            _target.Write<ushort>(entryAddr + (ulong)stateOffset, (ushort)flags);

            return;
        }

        // Find first free slot
        uint firstFree = length;
        for (uint i = 0; i < length; i++)
        {
            ulong entryAddr = entriesBase + (ulong)(i * entrySize);
            ushort state = _target.Read<ushort>(entryAddr + (ulong)stateOffset);
            if (state == CLRDATA_METHNOTIFY_NONE)
            {
                firstFree = i;
                break;
            }
        }

        if (firstFree >= capacity)
            throw new InvalidOperationException("JIT notification table is full");

        // Write new entry
        ulong newEntryAddr = entriesBase + (ulong)(firstFree * entrySize);
        WriteNUInt(newEntryAddr + (ulong)clrModuleOffset, module);
        _target.Write<uint>(newEntryAddr + (ulong)methodTokenOffset, methodToken);
        _target.Write<ushort>(newEntryAddr + (ulong)stateOffset, (ushort)flags);

        // Update length if we used a slot at the end
        if (firstFree >= length)
        {
            _target.Write<uint>(bookkeepingAddr + (ulong)methodTokenOffset, length + 1);
        }
    }

    uint INotifications.GetCodeNotification(TargetPointer module, uint methodToken)
    {
        Target.TypeInfo jitNotifType = _target.GetTypeInfo(DataType.JITNotification);
        int stateOffset = jitNotifType.Fields[nameof(Data.JITNotification.State)].Offset;
        int clrModuleOffset = jitNotifType.Fields[nameof(Data.JITNotification.ClrModule)].Offset;
        int methodTokenOffset = jitNotifType.Fields[nameof(Data.JITNotification.MethodToken)].Offset;
        uint entrySize = (uint)jitNotifType.Size!.Value;

        TargetPointer tablePointer = _target.ReadPointer(
            _target.ReadGlobalPointer(Constants.Globals.JITNotificationTable));

        if (tablePointer == TargetPointer.Null)
            throw new InvalidOperationException("JIT notification table is not initialized");

        ulong bookkeepingAddr = tablePointer;
        uint length = _target.Read<uint>(bookkeepingAddr + (ulong)methodTokenOffset);
        ulong entriesBase = tablePointer + entrySize;

        if (TryFindEntry(entriesBase, entrySize, stateOffset, clrModuleOffset, methodTokenOffset, length, module, methodToken, out uint foundIndex))
        {
            ulong entryAddr = entriesBase + (ulong)(foundIndex * entrySize);

            return _target.Read<ushort>(entryAddr + (ulong)stateOffset);
        }

        return CLRDATA_METHNOTIFY_NONE;
    }

    void INotifications.SetAllCodeNotifications(TargetPointer module, uint flags)
    {
        if (!IsValidMethodCodeNotification(flags))
            throw new ArgumentException("Invalid code notification flags", nameof(flags));

        Target.TypeInfo jitNotifType = _target.GetTypeInfo(DataType.JITNotification);
        int stateOffset = jitNotifType.Fields[nameof(Data.JITNotification.State)].Offset;
        int clrModuleOffset = jitNotifType.Fields[nameof(Data.JITNotification.ClrModule)].Offset;
        int methodTokenOffset = jitNotifType.Fields[nameof(Data.JITNotification.MethodToken)].Offset;
        uint entrySize = (uint)jitNotifType.Size!.Value;

        TargetPointer tablePointer = _target.ReadPointer(
            _target.ReadGlobalPointer(Constants.Globals.JITNotificationTable));

        if (tablePointer == TargetPointer.Null)
            throw new InvalidOperationException("JIT notification table is not initialized");

        ulong bookkeepingAddr = tablePointer;
        uint length = _target.Read<uint>(bookkeepingAddr + (ulong)methodTokenOffset);
        ulong entriesBase = tablePointer + entrySize;

        bool changed = false;
        for (uint i = 0; i < length; i++)
        {
            ulong entryAddr = entriesBase + (ulong)(i * entrySize);
            ushort state = _target.Read<ushort>(entryAddr + (ulong)stateOffset);
            if (state == CLRDATA_METHNOTIFY_NONE)
                continue;

            // If a module filter is specified, check it
            if (module != TargetPointer.Null)
            {
                TargetNUInt entryModule = _target.ReadNUInt(entryAddr + (ulong)clrModuleOffset);
                if (entryModule.Value != module.Value)
                    continue;
            }

            if (flags == CLRDATA_METHNOTIFY_NONE)
            {
                ClearEntry(entriesBase, entrySize, stateOffset, clrModuleOffset, methodTokenOffset, i);
            }
            else
            {
                _target.Write<ushort>(entryAddr + (ulong)stateOffset, (ushort)flags);
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
                ulong entryAddr = entriesBase + (ulong)((newLength - 1) * entrySize);
                ushort state = _target.Read<ushort>(entryAddr + (ulong)stateOffset);
                if (state != CLRDATA_METHNOTIFY_NONE)
                    break;
                newLength--;
            }

            _target.Write<uint>(bookkeepingAddr + (ulong)methodTokenOffset, newLength);
        }
    }

    private bool TryFindEntry(
        ulong entriesBase, uint entrySize,
        int stateOffset, int clrModuleOffset, int methodTokenOffset,
        uint length,
        TargetPointer module, uint methodToken,
        out uint index)
    {
        for (uint i = 0; i < length; i++)
        {
            ulong entryAddr = entriesBase + (ulong)(i * entrySize);
            ushort state = _target.Read<ushort>(entryAddr + (ulong)stateOffset);
            if (state == CLRDATA_METHNOTIFY_NONE)
                continue;

            TargetNUInt entryModule = _target.ReadNUInt(entryAddr + (ulong)clrModuleOffset);
            if (entryModule.Value != module.Value)
                continue;

            uint entryToken = _target.Read<uint>(entryAddr + (ulong)methodTokenOffset);
            if (entryToken != methodToken)
                continue;

            index = i;

            return true;
        }

        index = 0;

        return false;
    }

    private void ClearEntry(ulong entriesBase, uint entrySize, int stateOffset, int clrModuleOffset, int methodTokenOffset, uint index)
    {
        ulong entryAddr = entriesBase + (ulong)(index * entrySize);
        _target.Write<ushort>(entryAddr + (ulong)stateOffset, CLRDATA_METHNOTIFY_NONE);
        WriteNUInt(entryAddr + (ulong)clrModuleOffset, TargetPointer.Null);
        _target.Write<uint>(entryAddr + (ulong)methodTokenOffset, 0);
    }

    private void WriteNUInt(ulong address, TargetPointer value)
    {
        if (_target.PointerSize == 8)
            _target.Write<ulong>(address, value.Value);
        else
            _target.Write<uint>(address, (uint)value.Value);
    }

    private static bool IsValidMethodCodeNotification(uint flags)
    {
        return (flags & ~(CLRDATA_METHNOTIFY_GENERATED | CLRDATA_METHNOTIFY_DISCARDED)) == 0;
    }
}
