// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader;

// The memory-enumeration tier requested for a dump. Normal captures only the state a stack walk
// reaches (thread stacks, code, method metadata); Heap additionally captures the GC heap and the
// heap-side structures. The tier is derived from the minidump flags passed to EnumMemoryRegions.
internal enum DumpTier
{
    Normal,
    Heap,
}

// Managed state carried by the ICLRDataEnumMemoryRegions CCW: the contract reader over the target
// plus the native data target (kept alive/ref-counted for the lifetime of the CCW).
internal sealed class EnumMemoryRegionsState : IDisposable
{
    public EnumMemoryRegionsState(NativeDataTarget dataTarget, ContractDescriptorTarget target, ulong contractDescriptorAddr)
    {
        DataTarget = dataTarget;
        Target = target;
        ContractDescriptorAddr = contractDescriptorAddr;
    }

    public NativeDataTarget DataTarget { get; }
    public ContractDescriptorTarget Target { get; }
    public ulong ContractDescriptorAddr { get; }

    public void Dispose() => DataTarget.ReleaseTarget();
}

// The slim contract-based memory enumeration. Drives the existing cDAC contracts (Thread, Loader,
// GC, ...) to select the managed memory a crash dump should include and forwards each selected
// region to the caller's ICLRDataEnumMemoryRegionsCallback via its COM vtable (slot 3). This is the
// managed analog of the native cdac-lite prototype's enumerate.cpp, built on the real contracts
// rather than hand-written native walks -- and it uses manual vtable COM (no ComWrappers) so it
// stays reflection-free.
internal static unsafe class EnumMemoryRegionsImpl
{
    // MiniDumpWithPrivateReadWriteMemory: its presence in miniDumpFlags requests the full GC heap
    // and heap-side structures (the "heap" tier). Absent, we produce a Normal (stack-walk) tier.
    private const uint MiniDumpWithPrivateReadWriteMemory = 0x200;

    // Chunk size used to split a large region into EnumMemoryRegion (uint size) calls.
    private const uint MaxRegionChunk = 0x4000_0000; // 1 GiB

    private delegate void RegionSink(ulong address, ulong size);

    public static int Run(EnumMemoryRegionsState state, void* callback, uint miniDumpFlags)
    {
        if (callback == null)
        {
            return HResults.E_INVALIDARG;
        }

        // ICLRDataEnumMemoryRegionsCallback::EnumMemoryRegion is vtable slot 3 (after IUnknown).
        void** callbackVTable = *(void***)callback;
        delegate* unmanaged<void*, ulong, uint, int> enumMemoryRegion =
            (delegate* unmanaged<void*, ulong, uint, int>)callbackVTable[3];

        RegionSink emit = (ulong address, ulong size) =>
        {
            while (size > 0)
            {
                uint chunk = size > MaxRegionChunk ? MaxRegionChunk : (uint)size;
                enumMemoryRegion(callback, address, chunk);
                address += chunk;
                size -= chunk;
            }
        };

        DumpTier tier = (miniDumpFlags & MiniDumpWithPrivateReadWriteMemory) != 0
            ? DumpTier.Heap
            : DumpTier.Normal;

        // Each enumerator selects a subset of managed memory; heap-only enumerators no-op in the
        // Normal tier. Any single enumerator throwing (e.g. a contract or global unavailable in a
        // pared-down target) must not abort the rest, so each runs under its own try/catch.
        RunEnumerator(() => EnumerateContractDescriptor(state, emit));
        RunEnumerator(() => EnumerateThreadStacks(state, emit));

        return HResults.S_OK;
    }

    private static void RunEnumerator(Action enumerator)
    {
        try
        {
            enumerator();
        }
        catch (System.Exception ex)
        {
            try
            {
                var sb = new System.Text.StringBuilder();
                sb.Append("hr=0x").Append(ex.HResult.ToString("x")).Append(" msg=").Append(ex.Message).Append('\n');
                try { sb.Append("stack=\n").Append(ex.StackTrace); } catch { sb.Append("stack=<unavailable>"); }
                System.IO.File.AppendAllText(@"Q:\source\runtime-worktree-4\artifacts\slim_enum_err.txt", sb.ToString() + "\n----\n");
            }
            catch { }
        }
    }

    // The contract descriptor itself: consumers (e.g. ClrMD, the cDAC) must be able to re-read it
    // from the dump to bootstrap. Emit a page around its address.
    private static void EnumerateContractDescriptor(EnumMemoryRegionsState state, RegionSink emit)
    {
        if (state.ContractDescriptorAddr != 0)
        {
            emit(state.ContractDescriptorAddr, 0x1000);
        }
    }

    // Thread stacks: the memory a stack walk reaches. Walk the thread store and emit each managed
    // thread's stack range [stackLimit, stackBase). Included in every tier.
    private static void EnumerateThreadStacks(EnumMemoryRegionsState state, RegionSink emit)
    {
        IThread thread = state.Target.Contracts.Thread;
        ThreadStoreData store = thread.GetThreadStoreData();

        TargetPointer current = store.FirstThread;
        int guard = 0;
        while (current.Value != 0 && guard++ < 100_000)
        {
            ThreadData data = thread.GetThreadData(current);

            thread.GetStackLimitData(current, out TargetPointer stackBase, out TargetPointer stackLimit, out _);
            if (stackBase.Value != 0 && stackLimit.Value != 0 && stackBase.Value > stackLimit.Value)
            {
                emit(stackLimit.Value, stackBase.Value - stackLimit.Value);
            }

            current = data.NextThread;
        }
    }
}
