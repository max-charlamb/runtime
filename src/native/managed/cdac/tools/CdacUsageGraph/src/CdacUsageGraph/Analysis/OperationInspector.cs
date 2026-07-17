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
    /// If <paramref name="expr"/> is a <c>GetTypeInfo</c> call with a string name or enum
    /// member argument, returns the corresponding cDAC type name. The product enum overload
    /// forwards through <c>ToName()</c> to the string overload; Roslyn sees the source overload
    /// call before that body is inlined.
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
            if (arg.Parameter?.Type.TypeKind == TypeKind.Enum &&
                v is IFieldReferenceOperation f &&
                f.Field.ContainingType?.TypeKind == TypeKind.Enum)
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
        if (pr.Parent is ICompoundAssignmentOperation compoundAssignment &&
            ReferenceEquals(compoundAssignment.Target, pr))
            return UsageKind.ReadWrite;
        if (pr.Parent is IIncrementOrDecrementOperation increment &&
            ReferenceEquals(increment.Target, pr))
            return UsageKind.ReadWrite;
        return UsageKind.Read;
    }
}
