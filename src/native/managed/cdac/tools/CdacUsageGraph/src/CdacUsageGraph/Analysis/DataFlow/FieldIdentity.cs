// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>A (descriptor, field) pair identifying one `[Field]`/`[FieldAddress]` member.</summary>
internal readonly record struct FieldIdentity(DescriptorName Descriptor, string FieldName);
