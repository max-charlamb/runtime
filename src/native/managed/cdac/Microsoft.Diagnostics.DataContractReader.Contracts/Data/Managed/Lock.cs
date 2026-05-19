// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace Microsoft.Diagnostics.DataContractReader.Data.Managed;

[CdacType(nameof(DataType.Lock), ManagedFullName = "System.Threading.Lock")]
internal sealed partial class Lock : IData<Lock>
{
    [Field("State", "_state")]
    public uint State { get; }

    [Field("OwningThreadId", "_owningThreadId")]
    public int OwningThreadId { get; }

    [Field("RecursionCount", "_recursionCount")]
    public uint RecursionCount { get; }
}
