// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;

namespace Microsoft.Diagnostics.DataContractReader;

// Manual (function-pointer / vtable) COM interop for the slim dump reader.
//
// The slim reader deliberately avoids source-generated ComWrappers (GeneratedComInterface):
// the RCW/CCW paths resolve an interface's IID to its implementation via a runtime custom-attribute
// lookup (IIUnknownDerivedDetails.GetFromAttribute -> Type.GetCustomAttribute), which requires
// reflection metadata. That makes source-gen COM incompatible with full reflection-free NativeAOT
// (--reflectiondata:none). Doing the COM calls by hand through the vtable keeps the entire reader
// reflection-free (and drops the ComWrappers dependency), which is the biggest remaining size lever.
//
// Assumes the reader is the same architecture as its target (it always is -- it ships next to the
// runtime it inspects), so a single native calling convention applies.

internal static class Iids
{
    public static readonly Guid IUnknown = new("00000000-0000-0000-C000-000000000046");
    public static readonly Guid ICLRDataEnumMemoryRegions = new("471c35b4-7c2f-4ef0-a945-00f8c38056f1");
    public static readonly Guid ICLRContractLocator = new("17d5b8c6-34a9-407f-af4f-a930201d4e02");
}

internal static class ComHResults
{
    public const int E_NOINTERFACE = unchecked((int)0x80004002);
}

// Wraps a native ICLRDataTarget* and calls it through its COM vtable. Vtable slots follow
// clrdata.idl: IUnknown (0 QueryInterface, 1 AddRef, 2 Release), then GetMachineType(3),
// GetPointerSize(4), GetImageBase(5), ReadVirtual(6), WriteVirtual(7), GetTLSValue(8),
// SetTLSValue(9), GetCurrentThreadID(10), GetThreadContext(11), SetThreadContext(12), Request(13).
internal sealed unsafe class NativeDataTarget
{
    private readonly void* _target;

    public NativeDataTarget(void* target)
    {
        _target = target;
        AddRefTarget();
    }

    private static void** VTable(void* obj) => *(void***)obj;

    public int ReadVirtual(ulong address, Span<byte> buffer)
    {
        void** vt = VTable(_target);
        fixed (byte* p = buffer)
        {
            uint done;
            return ((delegate* unmanaged<void*, ulong, byte*, uint, uint*, int>)vt[6])(_target, address, p, (uint)buffer.Length, &done);
        }
    }

    public int WriteVirtual(ulong address, Span<byte> buffer)
    {
        void** vt = VTable(_target);
        fixed (byte* p = buffer)
        {
            uint done;
            return ((delegate* unmanaged<void*, ulong, byte*, uint, uint*, int>)vt[7])(_target, address, p, (uint)buffer.Length, &done);
        }
    }

    public int GetThreadContext(uint threadId, uint contextFlags, Span<byte> buffer)
    {
        void** vt = VTable(_target);
        fixed (byte* p = buffer)
        {
            return ((delegate* unmanaged<void*, uint, uint, uint, byte*, int>)vt[11])(_target, threadId, contextFlags, (uint)buffer.Length, p);
        }
    }

    public int SetThreadContext(uint threadId, ReadOnlySpan<byte> context)
    {
        void** vt = VTable(_target);
        fixed (byte* p = context)
        {
            return ((delegate* unmanaged<void*, uint, uint, byte*, int>)vt[12])(_target, threadId, (uint)context.Length, p);
        }
    }

    // Resolves the runtime's contract descriptor address by QueryInterface'ing the target for
    // ICLRContractLocator and calling GetContractDescriptor (its vtable slot 3, after IUnknown).
    public int GetContractDescriptor(out ulong contractAddress)
    {
        contractAddress = 0;
        void** vt = VTable(_target);
        Guid iid = Iids.ICLRContractLocator;
        void* locator;
        int hr = ((delegate* unmanaged<void*, Guid*, void**, int>)vt[0])(_target, &iid, &locator);
        if (hr < 0 || locator == null)
        {
            return hr < 0 ? hr : ComHResults.E_NOINTERFACE;
        }

        void** lvt = VTable(locator);
        ulong addr;
        hr = ((delegate* unmanaged<void*, ulong*, int>)lvt[3])(locator, &addr);
        ((delegate* unmanaged<void*, uint>)lvt[2])(locator); // Release
        if (hr >= 0)
        {
            contractAddress = addr;
        }
        return hr;
    }

    public void AddRefTarget() => ((delegate* unmanaged<void*, uint>)VTable(_target)[1])(_target);

    public void ReleaseTarget() => ((delegate* unmanaged<void*, uint>)VTable(_target)[2])(_target);
}

// A hand-built COM CCW exposing ICLRDataEnumMemoryRegions to the native caller (createdump/dbghelp).
// The native object layout is { void** vtable; long refCount; IntPtr gcHandle } where gcHandle
// points at the managed EnumMemoryRegionsState. The vtable holds four UnmanagedCallersOnly slots
// (IUnknown + EnumMemoryRegions), matching the ICLRDataEnumMemoryRegions COM layout.
internal static unsafe class EnumRegionsCcw
{
    private struct CcwInstance
    {
        public void** Vtable;
        public long RefCount;
        public IntPtr GcHandle;
    }

    private static readonly void** s_vtable = BuildVtable();

    private static void** BuildVtable()
    {
        void** vt = (void**)NativeMemory.Alloc(4, (nuint)sizeof(void*));
        vt[0] = (void*)(delegate* unmanaged<CcwInstance*, Guid*, void**, int>)&QueryInterface;
        vt[1] = (void*)(delegate* unmanaged<CcwInstance*, uint>)&AddRef;
        vt[2] = (void*)(delegate* unmanaged<CcwInstance*, uint>)&Release;
        vt[3] = (void*)(delegate* unmanaged<CcwInstance*, void*, uint, int, int>)&EnumMemoryRegions;
        return vt;
    }

    // Creates a CCW with an initial reference count of 1.
    public static void* Create(EnumMemoryRegionsState state)
    {
        CcwInstance* instance = (CcwInstance*)NativeMemory.Alloc((nuint)sizeof(CcwInstance));
        instance->Vtable = s_vtable;
        instance->RefCount = 1;
        instance->GcHandle = GCHandle.ToIntPtr(GCHandle.Alloc(state));
        return instance;
    }

    [UnmanagedCallersOnly]
    private static int QueryInterface(CcwInstance* self, Guid* iid, void** ppvObject)
    {
        if (ppvObject == null)
        {
            return HResults.E_POINTER;
        }
        if (*iid == Iids.IUnknown || *iid == Iids.ICLRDataEnumMemoryRegions)
        {
            Interlocked.Increment(ref self->RefCount);
            *ppvObject = self;
            return HResults.S_OK;
        }
        *ppvObject = null;
        return ComHResults.E_NOINTERFACE;
    }

    [UnmanagedCallersOnly]
    private static uint AddRef(CcwInstance* self) => (uint)Interlocked.Increment(ref self->RefCount);

    [UnmanagedCallersOnly]
    private static uint Release(CcwInstance* self)
    {
        long count = Interlocked.Decrement(ref self->RefCount);
        if (count == 0)
        {
            GCHandle handle = GCHandle.FromIntPtr(self->GcHandle);
            (handle.Target as EnumMemoryRegionsState)?.Dispose();
            handle.Free();
            NativeMemory.Free(self);
        }
        return (uint)count;
    }

    [UnmanagedCallersOnly]
    private static int EnumMemoryRegions(CcwInstance* self, void* callback, uint miniDumpFlags, int clrFlags)
    {
        try
        {
            if (GCHandle.FromIntPtr(self->GcHandle).Target is not EnumMemoryRegionsState state)
            {
                return HResults.E_FAIL;
            }
            return EnumMemoryRegionsImpl.Run(state, callback, miniDumpFlags);
        }
        catch (Exception ex)
        {
            int hr = ex.HResult;
            return hr < 0 ? hr : HResults.E_FAIL;
        }
    }
}
