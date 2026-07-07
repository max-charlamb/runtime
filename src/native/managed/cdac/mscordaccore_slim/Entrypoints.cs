// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Runtime.InteropServices;

namespace Microsoft.Diagnostics.DataContractReader;

// Entry points for the slim dump reader. It exports the classic DAC factory entry point
// (CLRDataCreateInstance) but only ever produces an ICLRDataEnumMemoryRegions -- just enough to
// drive crash-dump memory enumeration for a nearby runtime, without the Legacy SOS/DAC layer.
//
// All COM at this boundary is done by hand through vtables (see Interop.cs); the reader does not use
// source-generated ComWrappers so it can be compiled fully reflection-free.
internal static unsafe class SlimEntrypoints
{
    // Same export name and signature as the DAC's CLRDataCreateInstance in daccess.cpp.
    [UnmanagedCallersOnly(EntryPoint = "CLRDataCreateInstance")]
    private static int CLRDataCreateInstance(Guid* pIID, void* /*ICLRDataTarget*/ pLegacyTarget, void** iface)
    {
        if (pLegacyTarget == null || iface == null)
        {
            return HResults.E_INVALIDARG;
        }
        *iface = null;

        try
        {
            return CLRDataCreateInstanceCore(pIID, pLegacyTarget, iface);
        }
        catch (Exception ex)
        {
            try
            {
                string info = "hr=0x" + ex.HResult.ToString("x") + " msg=" + ex.Message +
                    " inner=" + (ex.InnerException is { } inner ? ("hr=0x" + inner.HResult.ToString("x") + " " + inner.Message) : "none");
                byte[] bytes = System.Text.Encoding.UTF8.GetBytes(info);
                using System.IO.FileStream fs = System.IO.File.Create(@"Q:\source\runtime-worktree-4\artifacts\slim_err2.txt");
                fs.Write(bytes, 0, bytes.Length);
            }
            catch { }
            int hr = ex.HResult;
            return hr < 0 ? hr : HResults.E_FAIL;
        }
    }

    private static int CLRDataCreateInstanceCore(Guid* pIID, void* pLegacyTarget, void** iface)
    {
        // Only ICLRDataEnumMemoryRegions (and IUnknown) are supported by the slim reader.
        if (*pIID != Iids.ICLRDataEnumMemoryRegions && *pIID != Iids.IUnknown)
        {
            return ComHResults.E_NOINTERFACE;
        }

        NativeDataTarget dataTarget = new(pLegacyTarget);

        int hr = dataTarget.GetContractDescriptor(out ulong contractAddress);
        if (hr < 0 || contractAddress == 0)
        {
            dataTarget.ReleaseTarget();
            return hr < 0 ? hr : HResults.E_FAIL;
        }

        if (!ContractDescriptorTarget.TryCreate(
            contractAddress,
            dataTarget.ReadVirtual,
            dataTarget.WriteVirtual,
            dataTarget.GetThreadContext,
            dataTarget.SetThreadContext,
            (ulong size, out ulong allocatedAddress) =>
            {
                allocatedAddress = 0;
                return HResults.E_NOTIMPL;
            },
            [Contracts.CoreCLRContracts.RegisterThreadOnly],
            out ContractDescriptorTarget? target))
        {
            dataTarget.ReleaseTarget();
            return HResults.E_FAIL;
        }

        EnumMemoryRegionsState state = new(dataTarget, target, contractAddress);
        // Create() returns a CCW with refcount 1; hand that reference to the caller.
        *iface = EnumRegionsCcw.Create(state);
        return HResults.S_OK;
    }
}
