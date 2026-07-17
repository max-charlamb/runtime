// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Model;

namespace CdacUsageGraph.Analysis.DataFlow;

internal sealed record FieldAccessEffect(FieldIdentity Field, UsageKind Usage) : CdacEffect;
