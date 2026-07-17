// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis.DataFlow.Framework;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.FlowAnalysis;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Analysis.DataFlow;

/// <summary>
/// Intraprocedural CFG analysis for cDAC provenance: builds a Roslyn <see cref="ControlFlowGraph"/>
/// for a method/constructor body and runs the generic <see cref="ForwardControlFlowSolver{TValue, TEffect}"/>
/// with a <see cref="CdacProvenanceTransfer"/>. This intentionally supports only local/capture flow
/// and inline producer calls; interprocedural summaries are layered on by <see cref="ProvenanceResolver"/>.
/// </summary>
internal sealed class ProvenanceDataFlowAnalysis
{
    private readonly CdacProvenanceTransfer _transfer;

    public ProvenanceDataFlowAnalysis(
        CdacApiSymbols apiSymbols,
        Func<IOperation, FiniteSetValue<DescriptorName>>? fallback = null,
        Func<
            IInvocationOperation,
            IReadOnlyDictionary<int, DescriptorProvenanceValue>,
            InvocationFlowResult?>? invocationResolver = null,
        Func<
            IObjectCreationOperation,
            IReadOnlyDictionary<int, DescriptorProvenanceValue>,
            InvocationFlowResult?>? objectCreationResolver = null) =>
        _transfer = new CdacProvenanceTransfer(apiSymbols, fallback, invocationResolver, objectCreationResolver);

    public FlowResult<DescriptorProvenanceValue, CdacEffect> Analyze(
        IOperation root,
        IReadOnlyDictionary<IParameterSymbol, DescriptorProvenanceValue>? parameterValues = null)
    {
        ControlFlowGraph graph = root switch
        {
            IMethodBodyOperation methodBody => ControlFlowGraph.Create(methodBody),
            IConstructorBodyOperation constructorBody => ControlFlowGraph.Create(constructorBody),
            _ => throw new ArgumentException(
                $"Unsupported CFG root operation '{root.Kind}'.", nameof(root)),
        };
        return Analyze(graph, parameterValues);
    }

    public FlowResult<DescriptorProvenanceValue, CdacEffect> Analyze(
        SyntaxNode declaration,
        SemanticModel semanticModel,
        IReadOnlyDictionary<IParameterSymbol, DescriptorProvenanceValue>? parameterValues = null)
    {
        ControlFlowGraph? graph = ControlFlowGraph.Create(declaration, semanticModel);
        if (graph is null)
            throw new ArgumentException(
                $"Could not create a CFG for '{declaration.Kind()}'.",
                nameof(declaration));
        return Analyze(graph, parameterValues);
    }

    private FlowResult<DescriptorProvenanceValue, CdacEffect> Analyze(
        ControlFlowGraph graph,
        IReadOnlyDictionary<IParameterSymbol, DescriptorProvenanceValue>? parameterValues) =>
        new ForwardControlFlowSolver<DescriptorProvenanceValue, CdacEffect>(_transfer)
            .Analyze(graph, parameterValues);
}
