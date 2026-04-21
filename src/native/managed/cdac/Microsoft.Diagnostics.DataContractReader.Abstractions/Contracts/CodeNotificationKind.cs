// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

/// <summary>
/// Kinds of JIT code notifications that can be requested for a given method.
/// These bits mirror the <c>CLRDataMethodCodeNotification</c> values in xclrdata.idl,
/// but the contract layer only exchanges this typed enum — COM wrappers translate
/// to/from the raw uint at the boundary.
/// </summary>
[Flags]
public enum CodeNotificationKind : uint
{
    None = 0,
    Generated = 1,
    Discarded = 2,
}
