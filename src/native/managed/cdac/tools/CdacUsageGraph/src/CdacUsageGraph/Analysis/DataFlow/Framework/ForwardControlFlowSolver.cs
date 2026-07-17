// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.FlowAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow.Framework;

/// <summary>
/// A generic forward, monotonic worklist dataflow solver over a Roslyn <see cref="ControlFlowGraph"/>.
/// It owns the CFG inputs/outputs, the worklist, same-state propagation to every successor, block
/// input joins, convergence, and the final exit state; all analysis-specific behavior (what an
/// operation evaluates to, and what effects it produces) lives in the injected
/// <see cref="IOperationTransfer{TValue, TEffect}"/>.
/// </summary>
internal sealed class ForwardControlFlowSolver<TValue, TEffect>
    where TValue : IFlowValue<TValue>
{
    private readonly IOperationTransfer<TValue, TEffect> _transfer;

    public ForwardControlFlowSolver(IOperationTransfer<TValue, TEffect> transfer) => _transfer = transfer;

    public FlowResult<TValue, TEffect> Analyze(
        ControlFlowGraph graph,
        IReadOnlyDictionary<IParameterSymbol, TValue>? parameterValues)
    {
        FlowResultSink<TValue, TEffect> sink = new();
        TValue returnValue = TValue.Bottom;
        FlowState<TValue>?[] inputs = new FlowState<TValue>?[graph.Blocks.Length];
        FlowState<TValue>?[] outputs = new FlowState<TValue>?[graph.Blocks.Length];
        Queue<BasicBlock> worklist = new();
        FlowState<TValue> entryState = new();
        if (parameterValues is not null)
        {
            foreach (KeyValuePair<IParameterSymbol, TValue> parameter in parameterValues)
                entryState[FlowSlot.ForSymbol(parameter.Key)] = parameter.Value;
        }
        inputs[graph.Blocks[0].Ordinal] = entryState;
        worklist.Enqueue(graph.Blocks[0]);

        while (worklist.Count > 0)
        {
            BasicBlock block = worklist.Dequeue();
            FlowState<TValue> state = inputs[block.Ordinal]!.Clone();
            foreach (IOperation operation in block.Operations)
                _transfer.Evaluate(operation, state, sink);
            TValue branchValue = block.BranchValue is not null
                ? _transfer.Evaluate(block.BranchValue, state, sink)
                : TValue.Bottom;
            if (block.FallThroughSuccessor?.Semantics == ControlFlowBranchSemantics.Return ||
                block.ConditionalSuccessor?.Semantics == ControlFlowBranchSemantics.Return)
                returnValue = returnValue.Join(branchValue);

            if (outputs[block.Ordinal]?.Equals(state) == true)
                continue;
            outputs[block.Ordinal] = state;

            Propagate(block.FallThroughSuccessor, state);
            Propagate(block.ConditionalSuccessor, state);
        }

        BasicBlock exitBlock = graph.Blocks[^1];
        FlowState<TValue> exitState =
            inputs[exitBlock.Ordinal]?.Clone() ?? new FlowState<TValue>();
        return new FlowResult<TValue, TEffect>(sink.Values, sink.Effects, returnValue, exitState);

        void Propagate(ControlFlowBranch? branch, FlowState<TValue> state)
        {
            if (branch?.Destination is not BasicBlock destination)
                return;

            if (inputs[destination.Ordinal] is null)
            {
                inputs[destination.Ordinal] = state.Clone();
                worklist.Enqueue(destination);
            }
            else if (inputs[destination.Ordinal]!.JoinFrom(state))
            {
                worklist.Enqueue(destination);
            }
        }
    }
}
