// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

internal readonly struct CodeNotifications_1 : ICodeNotifications
{
    private readonly Target _target;

    internal CodeNotifications_1(Target target)
    {
        _target = target;
    }

    void ICodeNotifications.SetCodeNotification(TargetPointer module, uint methodToken, CodeNotificationKind flags)
    {
        TableView? view = PrepareTable(allocateIfMissing: flags != CodeNotificationKind.None);
        if (view is null)
            return;

        if (flags == CodeNotificationKind.None)
        {
            if (TryFindEntry(view.Value, module, methodToken, out uint foundIndex))
            {
                Data.JITNotification entry = GetEntry(view.Value, foundIndex);
                entry.Clear();
                if (foundIndex == view.Value.Length - 1)
                {
                    view.Value.Bookkeeping.MethodToken = view.Value.Length - 1;
                }
            }

            return;
        }

        if (TryFindEntry(view.Value, module, methodToken, out uint existingIndex))
        {
            Data.JITNotification entry = GetEntry(view.Value, existingIndex);
            entry.State = (ushort)flags;

            return;
        }

        uint firstFree = view.Value.Length;
        for (uint i = 0; i < view.Value.Length; i++)
        {
            if (GetEntry(view.Value, i).IsFree)
            {
                firstFree = i;
                break;
            }
        }

        if (firstFree >= view.Value.Capacity)
            throw new InvalidOperationException("JIT notification table is full");

        GetEntry(view.Value, firstFree).WriteEntry(module, methodToken, (ushort)flags);

        if (firstFree >= view.Value.Length)
        {
            view.Value.Bookkeeping.MethodToken = view.Value.Length + 1;
        }
    }

    CodeNotificationKind ICodeNotifications.GetCodeNotification(TargetPointer module, uint methodToken)
    {
        TableView view = PrepareTable(allocateIfMissing: false)
            ?? throw new InvalidOperationException("JIT notification table not allocated");

        if (TryFindEntry(view, module, methodToken, out uint foundIndex))
        {
            return (CodeNotificationKind)GetEntry(view, foundIndex).State;
        }

        return CodeNotificationKind.None;
    }

    void ICodeNotifications.SetAllCodeNotifications(TargetPointer module, CodeNotificationKind flags)
    {
        TableView? maybeView = PrepareTable(allocateIfMissing: false);
        if (maybeView is null)
            return;

        TableView view = maybeView.Value;
        bool changed = false;
        for (uint i = 0; i < view.Length; i++)
        {
            Data.JITNotification entry = GetEntry(view, i);
            if (entry.IsFree)
                continue;

            if (module != TargetPointer.Null && entry.ClrModule.Value != module.Value)
                continue;

            if (flags == CodeNotificationKind.None)
            {
                entry.Clear();
            }
            else
            {
                entry.State = (ushort)flags;
            }

            changed = true;
        }

        if (changed && flags == CodeNotificationKind.None)
        {
            uint newLength = view.Length;
            while (newLength > 0 && GetEntry(view, newLength - 1).IsFree)
            {
                newLength--;
            }

            view.Bookkeeping.MethodToken = newLength;
        }
    }

    uint ICodeNotifications.GetCodeNotificationCapacity()
    {
        return _target.ReadGlobal<uint>(Constants.Globals.JITNotificationTableSize);
    }

    /// <summary>
    /// Snapshot of the prepared JIT notification table: the bookkeeping slot, the base
    /// address of the entry array, the current length and total capacity, and the entry
    /// stride. Produced by <see cref="PrepareTable"/>.
    /// </summary>
    private readonly struct TableView
    {
        public readonly Data.JITNotification Bookkeeping;
        public readonly ulong EntriesBase;
        public readonly uint EntrySize;
        public readonly uint Length;
        public readonly uint Capacity;

        public TableView(Data.JITNotification bookkeeping, ulong entriesBase, uint entrySize, uint length, uint capacity)
        {
            Bookkeeping = bookkeeping;
            EntriesBase = entriesBase;
            EntrySize = entrySize;
            Length = length;
            Capacity = capacity;
        }
    }

    /// <summary>
    /// Read (and optionally lazily allocate) the JIT notification table. Returns null if
    /// the table is not allocated and <paramref name="allocateIfMissing"/> is false.
    /// </summary>
    private TableView? PrepareTable(bool allocateIfMissing)
    {
        uint entrySize = GetEntrySize();
        TargetPointer tablePointer = ReadTablePointer();

        if (tablePointer == TargetPointer.Null)
        {
            if (!allocateIfMissing)
                return null;
            tablePointer = AllocateTable(entrySize);
        }

        Data.JITNotification bookkeeping = new(_target, tablePointer);
        uint length = bookkeeping.MethodToken;
        uint capacity = _target.ReadGlobal<uint>(Constants.Globals.JITNotificationTableSize);
        ulong entriesBase = tablePointer + entrySize;

        return new TableView(bookkeeping, entriesBase, entrySize, length, capacity);
    }

    private Data.JITNotification GetEntry(TableView view, uint index)
        => new(_target, new TargetPointer(view.EntriesBase + (ulong)(index * view.EntrySize)));

    private bool TryFindEntry(TableView view, TargetPointer module, uint methodToken, out uint index)
    {
        for (uint i = 0; i < view.Length; i++)
        {
            Data.JITNotification entry = GetEntry(view, i);
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
        if (_target.PointerSize == 8)
            _target.Write<ulong>(globalAddr.Value, tablePointer.Value);
        else
            _target.Write<uint>(globalAddr.Value, (uint)tablePointer.Value);

        return tablePointer;
    }
}
