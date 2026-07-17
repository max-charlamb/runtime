// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// A generic lattice value contract for forward dataflow analyses: every value type has a
/// canonical bottom element and a commutative, idempotent, monotonic join operation used to merge
/// values arriving from different control-flow predecessors.
/// </summary>
internal interface IFlowValue<TSelf> : IEquatable<TSelf>
    where TSelf : IFlowValue<TSelf>
{
    static abstract TSelf Bottom { get; }

    TSelf Join(TSelf other);
}
