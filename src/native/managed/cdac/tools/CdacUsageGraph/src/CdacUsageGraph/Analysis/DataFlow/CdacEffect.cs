// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// A side effect discovered while evaluating a cDAC provenance flow: either a
/// <see cref="FieldAccessEffect"/> (a `[Field]`/`[FieldAddress]` read or write) or a
/// <see cref="GlobalAccessEffect"/> (a `ReadGlobal*`/`TryReadGlobal*` access). Kept as one tagged
/// base so <see cref="Framework.FlowResult{TValue, TEffect}"/> and
/// <see cref="Framework.IFlowResultSink{TValue, TEffect}"/> stay generic, while consumers (e.g.
/// <see cref="UsageWalker"/>) recover the concrete kind via pattern matching.
/// </summary>
internal abstract record CdacEffect;
