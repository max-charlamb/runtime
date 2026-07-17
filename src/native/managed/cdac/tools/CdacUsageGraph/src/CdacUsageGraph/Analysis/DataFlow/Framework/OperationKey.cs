// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Text;

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// A syntax-position-based key for remembering the flow value(s) evaluated for an
/// <see cref="IOperation"/>. Repeated evaluations of the same syntax (e.g. across worklist
/// iterations, or a shared root re-analyzed for different operation kinds) are joined rather than
/// overwritten by callers that key their per-operation maps with this type.
/// </summary>
internal readonly record struct OperationKey(
    SyntaxTree Tree,
    TextSpan Span,
    OperationKind Kind)
{
    public static OperationKey Create(IOperation operation) =>
        new(operation.Syntax.SyntaxTree, operation.Syntax.Span, operation.Kind);
}
