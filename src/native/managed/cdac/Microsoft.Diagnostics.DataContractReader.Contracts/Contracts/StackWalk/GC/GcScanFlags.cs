// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;

namespace Microsoft.Diagnostics.DataContractReader.Contracts.StackWalkHelpers;

[Flags]
internal enum GcScanFlags
{
    None = 0x0,
    GC_CALL_INTERIOR = 0x1,
    GC_CALL_PINNED = 0x2,

    // cDAC-private sentinel: this StackRefData is not a real GC reference but
    // a marker that an explicit Frame at `Source` was deliberately skipped by
    // the cDAC because the code path required is not implemented yet.
    CDAC_DEFERRED_FRAME = 0x40000000,

    // cDAC-private sentinel: this StackRefData marks not just a single deferred
    // Frame but the entire upstream walk from this point on as deferred. The
    // cDAC's walker cannot reliably continue past this Frame (e.g. on x86 where
    // EBP-chain unwinding depends on a cbStackPop the cDAC cannot compute),
    // so any runtime-reported frame the cDAC doesn't see should be treated as
    // a known not-implemented case rather than a mismatch.
    CDAC_DEFERRED_WALK = 0x20000000,
}
