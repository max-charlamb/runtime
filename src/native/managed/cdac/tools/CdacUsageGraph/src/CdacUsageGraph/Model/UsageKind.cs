// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Model;

/// <summary>How a contract uses a Data descriptor field. The member names are emitted verbatim in
/// the reports, so they must not be renamed without updating the expected output.</summary>
public enum UsageKind
{
    /// <summary>The field's value is read (e.g. <c>thread.Frame</c>).</summary>
    Read,

    /// <summary>The field is written (an assignment target).</summary>
    Write,

    /// <summary>The field is read-modify-written (compound assignment / increment).</summary>
    ReadWrite,

    /// <summary>Only the field's offset is referenced (e.g. <c>nameof(Data.Thread.Frame)</c> or
    /// a raw-string <c>Fields["..."]</c> lookup), not its value.</summary>
    OffsetLookup,
}
