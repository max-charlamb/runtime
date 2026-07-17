// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// A cDAC descriptor name (the first `[CdacType]` name of a Data type, e.g. "Widget"). Kept as a
/// distinct type rather than a raw <see cref="string"/> so the provenance domain can't
/// accidentally mix descriptor names with other kinds of strings (field names, global names, ...).
/// </summary>
internal readonly record struct DescriptorName(string Value);
