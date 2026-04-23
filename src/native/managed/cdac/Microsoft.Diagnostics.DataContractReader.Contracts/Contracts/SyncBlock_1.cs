// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

internal readonly struct SyncBlock_1 : ISyncBlock
{
    private const string LockStateName = "_state";
    private const string LockOwningThreadIdName = "_owningThreadId";
    private const string LockRecursionCountName = "_recursionCount";
    private const string LockName = "Lock";
    private const string LockNamespace = "System.Threading";
    private readonly Target _target;
    private readonly TargetPointer _syncTableEntries;
    private readonly ulong _syncBlockLinkOffset;

    internal SyncBlock_1(Target target)
    {
        _target = target;
        _syncTableEntries = target.ReadPointer(target.ReadGlobalPointer(Constants.Globals.SyncTableEntries));
        _syncBlockLinkOffset = (ulong)target.GetTypeInfo(DataType.SyncBlock).Fields[nameof(Data.SyncBlock.LinkNext)].Offset;
    }

    public TargetPointer GetSyncBlock(uint index)
    {
        Data.SyncTableEntry ste = _target.ProcessedData.GetOrAdd<Data.SyncTableEntry>(_syncTableEntries + index * _target.GetTypeInfo(DataType.SyncTableEntry).Size!.Value);
        return ste.SyncBlock?.Address ?? TargetPointer.Null;
    }

    public TargetPointer GetSyncBlockObject(uint index)
    {
        Data.SyncTableEntry ste = _target.ProcessedData.GetOrAdd<Data.SyncTableEntry>(_syncTableEntries + index * _target.GetTypeInfo(DataType.SyncTableEntry).Size!.Value);
        return ste.Object?.Address ?? TargetPointer.Null;
    }

    public bool IsSyncBlockFree(uint index)
    {
        Data.SyncTableEntry ste = _target.ProcessedData.GetOrAdd<Data.SyncTableEntry>(_syncTableEntries + index * _target.GetTypeInfo(DataType.SyncTableEntry).Size!.Value);
        return (ste.Object?.Address & 1) != 0;
    }

    public uint GetSyncBlockCount()
    {
        TargetPointer syncBlockCache = _target.ReadPointer(_target.ReadGlobalPointer(Constants.Globals.SyncBlockCache));
        Data.SyncBlockCache cache = _target.ProcessedData.GetOrAdd<Data.SyncBlockCache>(syncBlockCache);
        return cache.FreeSyncTableIndex - 1;
    }

    public bool TryGetLockInfo(TargetPointer syncBlock, out uint owningThreadId, out uint recursion)
    {
        owningThreadId = 0;
        recursion = 0;
        Data.SyncBlock sb = _target.ProcessedData.GetOrAdd<Data.SyncBlock>(syncBlock);

        if (sb.Lock != null)
        {
            ManagedTypeInfo lockType = _target.Contracts.ManagedTypeLayout.GetTypeInfo(LockNamespace, LockName);
            TargetPointer lockObjPtr = sb.Lock.Object;
            uint state = _target.ReadField<uint>(lockObjPtr, lockType, LockStateName);
            bool monitorHeld = (state & 1) != 0;
            if (monitorHeld)
            {
                owningThreadId = (uint)_target.ReadField<int>(lockObjPtr, lockType, LockOwningThreadIdName);
                recursion = _target.ReadField<uint>(lockObjPtr, lockType, LockRecursionCountName);
            }
            return monitorHeld;
        }

        else if (sb.ThinLock != 0)
        {
            owningThreadId = sb.ThinLock & _target.ReadGlobal<uint>(Constants.Globals.SyncBlockMaskLockThreadId);
            bool monitorHeld = owningThreadId != 0;
            if (monitorHeld)
            {
                recursion = (sb.ThinLock & _target.ReadGlobal<uint>(Constants.Globals.SyncBlockMaskLockRecursionLevel)) >> (int)_target.ReadGlobal<uint>(Constants.Globals.SyncBlockRecursionLevelShift);
            }
            return monitorHeld;
        }

        else
        {
            return false;
        }
    }

    public uint GetAdditionalThreadCount(TargetPointer syncBlock)
    {
        // TODO: read conditional weak table to get additional thread count
        return 0;
    }

    public TargetPointer GetSyncBlockFromCleanupList()
    {
        TargetPointer syncBlockCache = _target.ReadPointer(_target.ReadGlobalPointer(Constants.Globals.SyncBlockCache));
        Data.SyncBlockCache cache = _target.ProcessedData.GetOrAdd<Data.SyncBlockCache>(syncBlockCache);
        TargetPointer cleanupBlockList = cache.CleanupBlockList;
        if (cleanupBlockList == TargetPointer.Null)
            return TargetPointer.Null;
        return new TargetPointer(cleanupBlockList.Value - _syncBlockLinkOffset);
    }

    public TargetPointer GetNextSyncBlock(TargetPointer syncBlock)
    {
        Data.SyncBlock sb = _target.ProcessedData.GetOrAdd<Data.SyncBlock>(syncBlock);
        if (sb.LinkNext == TargetPointer.Null)
            return TargetPointer.Null;
        return new TargetPointer(sb.LinkNext.Value - _syncBlockLinkOffset);
    }

    public bool GetBuiltInComData(TargetPointer syncBlock, out TargetPointer rcw, out TargetPointer ccw, out TargetPointer ccf)
    {
        rcw = TargetPointer.Null;
        ccw = TargetPointer.Null;
        ccf = TargetPointer.Null;

        Data.SyncBlock sb = _target.ProcessedData.GetOrAdd<Data.SyncBlock>(syncBlock);
        Data.InteropSyncBlockInfo? interopInfo = sb.InteropInfo;
        if (interopInfo == null)
            return false;

        rcw = interopInfo.RCW & ~1ul;
        ccw = interopInfo.CCW == 1 ? TargetPointer.Null : interopInfo.CCW;
        ccf = interopInfo.CCF == 1 ? TargetPointer.Null : interopInfo.CCF;
        return rcw != TargetPointer.Null || ccw != TargetPointer.Null || ccf != TargetPointer.Null;
    }
}
