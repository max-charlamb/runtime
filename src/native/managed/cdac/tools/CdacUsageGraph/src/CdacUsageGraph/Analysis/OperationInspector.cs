// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Analysis;

/// <summary>Stateless helpers for inspecting <see cref="IOperation"/> shapes.</summary>
internal static class OperationInspector
{
    /// <summary>Strips implicit conversion wrappers.</summary>
    public static IOperation Unwrap(IOperation op)
    {
        while (op is IConversionOperation { Operand: { } operand })
            op = operand;
        return op;
    }

    /// <summary>
    /// If <paramref name="expr"/> is a <c>GetTypeInfo(DataType.X)</c> / <c>GetTypeInfo("X")</c>
    /// call, returns the DataType name.
    /// </summary>
    public static string? InlineGetTypeInfoName(IOperation? expr)
    {
        if (expr is null)
            return null;
        expr = Unwrap(expr);
        if (expr is not IInvocationOperation inv || inv.TargetMethod.Name != CdacSymbols.GetTypeInfoMethodName)
            return null;
        foreach (IArgumentOperation arg in inv.Arguments)
        {
            IOperation v = Unwrap(arg.Value);
            if (v is IFieldReferenceOperation f && f.Field.ContainingType?.Name == CdacSymbols.DataTypeEnumName)
                return f.Field.Name;
            if (v.ConstantValue is { HasValue: true, Value: string s })
                return s;
        }
        return null;
    }

    /// <summary>The local/field/parameter symbol an assignment target refers to.</summary>
    public static ISymbol? TargetSymbol(IOperation t) => Unwrap(t) switch
    {
        ILocalReferenceOperation l => l.Local,
        IFieldReferenceOperation f => f.Field,
        IParameterReferenceOperation p => p.Parameter,
        _ => null,
    };

    /// <summary>
    /// Given a <c>.Fields</c> property reference, extracts the constant string key from an
    /// indexer (<c>Fields["x"]</c>) or a <c>ContainsKey</c>/<c>TryGetValue(..)</c> call.
    /// </summary>
    public static string? ExtractFieldsKey(IOperation fieldsRef)
    {
        IOperation? keyOp = fieldsRef.Parent switch
        {
            IPropertyReferenceOperation idx when idx.Property.IsIndexer && idx.Arguments.Length >= 1
                => idx.Arguments[0].Value,
            IInvocationOperation call when call.Arguments.Length >= 1
                && call.TargetMethod.Name is "ContainsKey" or "TryGetValue"
                => call.Arguments[0].Value,
            _ => null,
        };
        if (keyOp is null)
            return null;
        keyOp = Unwrap(keyOp);
        return keyOp.ConstantValue is { HasValue: true, Value: string s } ? s : null;
    }

    /// <summary>
    /// Classifies a Data-property reference: <c>nameof(..)</c> is an offset lookup, an assignment
    /// target is a write, an increment/compound assignment is a read-write, else a read.
    /// </summary>
    public static UsageKind ClassifyPropertyRef(IPropertyReferenceOperation pr)
    {
        for (IOperation? p = pr.Parent; p is not null; p = p.Parent)
        {
            if (p is INameOfOperation)
                return UsageKind.OffsetLookup;
            if (p is IArgumentOperation or IInvocationOperation)
                break;
        }
        if (pr.Parent is ISimpleAssignmentOperation sa && ReferenceEquals(sa.Target, pr))
            return UsageKind.Write;
        if (pr.Parent is ICompoundAssignmentOperation or IIncrementOrDecrementOperation)
            return UsageKind.ReadWrite;
        return UsageKind.Read;
    }
}
