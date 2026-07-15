// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace CdacUsageGraph.Discovery;

/// <summary>Semantic relationship helpers for Roslyn symbols.</summary>
internal static class SymbolExtensions
{
    /// <summary>
    /// Returns whether C# permits an implicit conversion from <paramref name="source"/> to
    /// <paramref name="target"/>. Covers identity, class/interface inheritance, implemented
    /// interfaces, and applicable generic/variance conversions.
    /// </summary>
    public static bool IsAssignableTo(
        this CSharpCompilation compilation,
        ITypeSymbol source,
        ITypeSymbol target) =>
        compilation.ClassifyConversion(source, target).IsImplicit;
}
