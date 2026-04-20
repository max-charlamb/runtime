// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;

namespace Microsoft.Diagnostics.DataContractReader.Contracts;

/// <summary>
/// Contract for reading and writing the JIT code notification table in the target process.
/// The table is an allowlist of (module, method token) pairs that causes the runtime to
/// raise <c>DEBUG_CODE_NOTIFICATION</c> events when the specified methods are JIT-compiled
/// or discarded.
/// </summary>
public interface ICodeNotifications : IContract
{
    static string IContract.Name { get; } = nameof(CodeNotifications);

    /// <summary>
    /// Set the notification flags for a single (module, methodToken) pair.
    /// If the in-target table has not been allocated yet, lazily allocates it when
    /// <paramref name="flags"/> is non-zero.
    /// </summary>
    void SetCodeNotification(TargetPointer module, uint methodToken, uint flags) => throw new NotImplementedException();

    /// <summary>
    /// Get the notification flags for a single (module, methodToken) pair.
    /// </summary>
    uint GetCodeNotification(TargetPointer module, uint methodToken) => throw new NotImplementedException();

    /// <summary>
    /// Set notification flags for all methods in a module, or all methods if module is null.
    /// </summary>
    void SetAllCodeNotifications(TargetPointer module, uint flags) => throw new NotImplementedException();

    /// <summary>
    /// Returns the total capacity of the JIT notification table (the maximum number of entries
    /// that can be stored, excluding the bookkeeping slot). Used by callers to reject batch
    /// requests that cannot possibly fit.
    /// </summary>
    uint GetCodeNotificationCapacity() => throw new NotImplementedException();
}

public readonly struct CodeNotifications : ICodeNotifications
{
    // Everything throws NotImplementedException
}
