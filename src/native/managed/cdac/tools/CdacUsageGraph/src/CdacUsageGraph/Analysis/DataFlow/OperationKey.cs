// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Text;

namespace CdacUsageGraph.Analysis.DataFlow;

internal readonly record struct OperationKey(
    SyntaxTree Tree,
    TextSpan Span,
    OperationKind Kind)
{
    public static OperationKey Create(IOperation operation) =>
        new(operation.Syntax.SyntaxTree, operation.Syntax.Span, operation.Kind);
}
