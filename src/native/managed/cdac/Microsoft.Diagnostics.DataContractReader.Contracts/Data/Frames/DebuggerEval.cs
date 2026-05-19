// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.Diagnostics.DataContractReader.Generated;

namespace Microsoft.Diagnostics.DataContractReader.Data;

[CdacType(nameof(DataType.DebuggerEval))]
internal partial class DebuggerEval : IData<DebuggerEval>
{
    [FieldAddress]
    public TargetPointer TargetContext { get; }

    [Field] public bool EvalUsesHijack { get; }
}
