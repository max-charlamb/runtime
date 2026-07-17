// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

internal sealed record InvocationFlowResult(
    ProvenanceValue ReturnValue,
    IReadOnlyDictionary<int, ProvenanceValue> OutRefValues,
    IReadOnlyCollection<FieldAccessEffect> Effects,
    IReadOnlyCollection<GlobalAccessEffect> GlobalEffects);
