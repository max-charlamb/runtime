// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace Microsoft.Diagnostics.DataContractReader.Contracts.StackWalkHelpers;

// Signals from a frame handler that the cDAC cannot reliably continue the
// stack walk past the current Frame. The stack walker catches this, records
// the walk as deferred via GcScanContext.RecordDeferredWalk, and continues.
internal sealed class DeferredWalkException : System.Exception { }
