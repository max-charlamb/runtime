// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

namespace CdacUsageGraph.Compilation;

/// <summary>
/// Immutable Roslyn snapshots for the cDAC analysis universe. Discovery remains rooted in the
/// Contracts compilation; additional source projects are retained so the DFA can inspect their
/// method bodies and build CFGs instead of summarizing wrappers by name.
/// </summary>
internal sealed class CdacAnalysisWorkspace
{
    private readonly Dictionary<SyntaxTree, CSharpCompilation> _treeOwners;

    public CdacAnalysisWorkspace(
        CSharpCompilation contracts,
        IReadOnlyList<CSharpCompilation> analyzableCompilations)
    {
        Contracts = contracts;
        AnalyzableCompilations = analyzableCompilations;
        _treeOwners = new Dictionary<SyntaxTree, CSharpCompilation>();

        foreach (CSharpCompilation compilation in analyzableCompilations)
        {
            foreach (SyntaxTree tree in compilation.SyntaxTrees)
                _treeOwners.Add(tree, compilation);
        }
    }

    /// <summary>The Contracts compilation used for Data and registration discovery.</summary>
    public CSharpCompilation Contracts { get; }

    /// <summary>Source compilations whose operation bodies the DFA may inspect.</summary>
    public IReadOnlyList<CSharpCompilation> AnalyzableCompilations { get; }

    public bool IsAnalyzable(ISymbol? symbol) =>
        symbol is not null &&
        symbol.OriginalDefinition.DeclaringSyntaxReferences.Any(
            reference => _treeOwners.ContainsKey(reference.SyntaxTree));

    public bool TryGetSemanticModel(SyntaxTree tree, out SemanticModel model)
    {
        if (_treeOwners.TryGetValue(tree, out CSharpCompilation? compilation))
        {
            model = compilation.GetSemanticModel(tree);
            return true;
        }

        model = null!;
        return false;
    }
}
