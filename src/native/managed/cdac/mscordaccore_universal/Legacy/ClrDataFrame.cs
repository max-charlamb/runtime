// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.Marshalling;
using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader.Legacy;

[GeneratedComClass]
internal sealed unsafe partial class ClrDataFrame : IXCLRDataFrame
{
    private readonly Target _target;
    private readonly IStackWalk _stackWalk;
    private readonly IStackDataFrameHandle _dataFrameHandle;

    private readonly IXCLRDataFrame? _legacyImpl;

    public ClrDataFrame(
        Target target,
        IStackDataFrameHandle dataFrameHandle,
        IXCLRDataFrame? legacyImpl)
    {
        _target = target;
        _stackWalk = target.Contracts.StackWalk;

        _dataFrameHandle = dataFrameHandle;
        _legacyImpl = legacyImpl;
    }

    int IXCLRDataFrame.GetFrameType(CLRDataSimpleFrameType* simpleType, CLRDataDetailedFrameType* detailedType)
        => _legacyImpl is not null ? _legacyImpl.GetFrameType(simpleType, detailedType) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetContext(uint contextFlags, uint contextBufSize, uint* contextSize, byte[] contextBuf)
        => _legacyImpl is not null ? _legacyImpl.GetContext(contextFlags, contextBufSize, contextSize, contextBuf) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetAppDomain(void** appDomain)
        => _legacyImpl is not null ? _legacyImpl.GetAppDomain(appDomain) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetNumArguments(uint* numArgs)
    {
        int hr = HResults.S_OK;

        try
        {
            TargetPointer methodDescPtr = _stackWalk.GetMethodDescPointer(_dataFrameHandle);
            if (methodDescPtr == TargetPointer.Null)
                throw new ReturnHResultException(unchecked((int)0x80004002 /*E_NOINTERFACE*/));


        }
        catch (System.Exception ex)
        {
            hr = ex.HResult;
        }

#if DEBUG
        if (_legacyImpl is not null)
        {
            uint numArgsLocal = 0;
            int hrLocal = _legacyImpl.GetNumArguments(&numArgsLocal);

            Debug.Assert(hrLocal == hr, $"cDAC: {hr:x}, DAC: {hrLocal:x}");
            if (hr == HResults.S_OK)
            {
                Debug.Assert(numArgsLocal == *numArgs, $"cDAC: {*numArgs}, DAC: {numArgsLocal}");
            }
        }
#endif

        return hr;
    }

    int IXCLRDataFrame.GetArgumentByIndex(uint index, void** arg, uint bufLen, uint* nameLen, char[] name)
        => _legacyImpl is not null ? _legacyImpl.GetArgumentByIndex(index, arg, bufLen, nameLen, name) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetNumLocalVariables(uint* numLocals)
        => _legacyImpl is not null ? _legacyImpl.GetNumLocalVariables(numLocals) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetLocalVariableByIndex(uint index, void** localVariable, uint bufLen, uint* nameLen, char[] name)
        => _legacyImpl is not null ? _legacyImpl.GetLocalVariableByIndex(index, localVariable, bufLen, nameLen, name) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetCodeName(uint flags, uint bufLen, uint* nameLen, char[] nameBuf)
        => _legacyImpl is not null ? _legacyImpl.GetCodeName(flags, bufLen, nameLen, nameBuf) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetMethodInstance(void** method)
        => _legacyImpl is not null ? _legacyImpl.GetMethodInstance(method) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.Request(uint reqCode, uint inBufferSize, byte* inBuffer, uint outBufferSize, byte* outBuffer)
        => _legacyImpl is not null ? _legacyImpl.Request(reqCode, inBufferSize, inBuffer, outBufferSize, outBuffer) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetNumTypeArguments(uint* numTypeArgs)
        => _legacyImpl is not null ? _legacyImpl.GetNumTypeArguments(numTypeArgs) : HResults.E_NOTIMPL;

    int IXCLRDataFrame.GetTypeArgumentByIndex(uint index, void** typeArg)
        => _legacyImpl is not null ? _legacyImpl.GetTypeArgumentByIndex(index, typeArg) : HResults.E_NOTIMPL;
}
