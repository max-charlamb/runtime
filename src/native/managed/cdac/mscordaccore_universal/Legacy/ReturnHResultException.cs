// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;

namespace Microsoft.Diagnostics.DataContractReader.Legacy;

/// <summary>
/// Exception that wraps an HResult value to be returned from COM interop methods.
/// </summary>
internal sealed class ReturnHResultException : Exception
{
    public ReturnHResultException(int hresult) : base($"Operation failed with HResult 0x{hresult:X8}")
    {
        HResult = hresult;
    }
}
