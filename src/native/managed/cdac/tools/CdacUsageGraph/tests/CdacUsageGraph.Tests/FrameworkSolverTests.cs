// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis;
using CdacUsageGraph.Analysis.DataFlow.Framework;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.FlowAnalysis;
using Microsoft.CodeAnalysis.Operations;
using Xunit;

namespace CdacUsageGraph.Tests;

/// <summary>
/// Focused unit tests for the generic <c>Analysis.DataFlow.Framework</c> primitives, independent
/// of any cDAC-specific semantics. A minimal <see cref="ToyTransfer"/> assigns string-literal
/// identities (as <see cref="FiniteSetValue{T}"/> of <see cref="string"/>) to locals, parameters,
/// and flow captures, and records every string argument passed to a `Effects.Effect(...)` call as
/// an effect, so <see cref="ForwardControlFlowSolver{TValue, TEffect}"/> behavior (straight-line
/// flow, branch joins, loop convergence, and effect/value accumulation across worklist iterations)
/// can be exercised without any of <c>CdacProvenanceTransfer</c>'s API-recognition logic.
/// </summary>
public sealed class FrameworkSolverTests
{
    [Fact]
    public void FiniteSetValuesCannotBeMutated()
    {
        FiniteSetValue<string> value = FiniteSetValue<string>.Known("A");

        Assert.Throws<NotSupportedException>(
            () => ((ISet<string>)value.Values).Add("B"));
        Assert.Equal(["A"], value.Values);
    }

    [Fact]
    public void SolverPropagatesStraightLineAssignments()
    {
        const string source = """
            namespace Toy
            {
                public sealed class User
                {
                    public string Read()
                    {
                        string a = "A";
                        string b = a;
                        return b;
                    }
                }
            }
            """;
        (FlowResult<FiniteSetValue<string>, string> result, _, _) = Analyze(source, "Read");

        Assert.Equal(["A"], result.ReturnValue.Values.OrderBy(v => v, StringComparer.Ordinal));
    }

    [Fact]
    public void SolverJoinsDiamondBranchValues()
    {
        const string source = """
            namespace Toy
            {
                public sealed class User
                {
                    public string Read(bool condition)
                    {
                        string x;
                        if (condition)
                            x = "First";
                        else
                            x = "Second";
                        return x;
                    }
                }
            }
            """;
        (FlowResult<FiniteSetValue<string>, string> result, _, _) = Analyze(source, "Read");

        Assert.Equal(
            ["First", "Second"],
            result.ReturnValue.Values.OrderBy(v => v, StringComparer.Ordinal));
    }

    [Fact]
    public void SolverConvergesLoopFixedPointAndRetainsEffectsFromEveryIteration()
    {
        const string source = """
            namespace Toy
            {
                public static class Effects
                {
                    public static void Effect(string value) { }
                }

                public sealed class User
                {
                    public string Read(bool repeat)
                    {
                        string x = "First";
                        while (repeat)
                        {
                            Effects.Effect(x);
                            x = "Second";
                        }
                        return x;
                    }
                }
            }
            """;
        (FlowResult<FiniteSetValue<string>, string> result, _, _) = Analyze(source, "Read");

        // The back-edge re-enters the loop body with the joined ["First", "Second"] state, so the
        // loop condition block (and thus the final return) converges to both names.
        Assert.Equal(
            ["First", "Second"],
            result.ReturnValue.Values.OrderBy(v => v, StringComparer.Ordinal));

        // The first loop iteration observes only "First" and the second (converged) iteration
        // observes ["First", "Second"]; both must be retained rather than only the final pass.
        Assert.Equal(
            ["First", "Second"],
            result.Effects.OrderBy(v => v, StringComparer.Ordinal));
    }

    [Fact]
    public void SolverPropagatesFlowCaptureIdentitiesThroughConditionalExpression()
    {
        const string source = """
            namespace Toy
            {
                public sealed class User
                {
                    public string Read(bool condition)
                    {
                        string result = condition ? "First" : "Second";
                        return result;
                    }
                }
            }
            """;
        (FlowResult<FiniteSetValue<string>, string> result, _, _) = Analyze(source, "Read");

        // The ternary's two arms are each captured into the same CaptureId in different blocks
        // (IFlowCaptureOperation); the join block reads it back (IFlowCaptureReferenceOperation)
        // and assigns it to the local, which the final `return result;` observes joined.
        Assert.Equal(
            ["First", "Second"],
            result.ReturnValue.Values.OrderBy(v => v, StringComparer.Ordinal));
    }

    private static (
        FlowResult<FiniteSetValue<string>, string> Result,
        IMethodSymbol Method,
        SemanticModel Model) Analyze(string source, string methodName)
    {
        CSharpCompilation compilation = CSharpCompilation.Create(
            "ToyFrameworkTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        IMethodSymbol method = compilation.GetTypeByMetadataName("Toy.User")!
            .GetMembers(methodName)
            .OfType<IMethodSymbol>()
            .Single();
        SemanticModel model = compilation.GetSemanticModel(
            method.DeclaringSyntaxReferences[0].SyntaxTree);
        SyntaxNode declaration = method.DeclaringSyntaxReferences[0].GetSyntax();
        ControlFlowGraph graph = ControlFlowGraph.Create(declaration, model) ??
            throw new InvalidOperationException($"Could not create a CFG for '{methodName}'.");

        ForwardControlFlowSolver<FiniteSetValue<string>, string> solver = new(new ToyTransfer());
        return (solver.Analyze(graph, null), method, model);
    }

    private static IEnumerable<MetadataReference> RuntimeReferences()
    {
        string tpa = (string)AppContext.GetData("TRUSTED_PLATFORM_ASSEMBLIES")!;
        foreach (string path in tpa.Split(Path.PathSeparator))
        {
            if (path.EndsWith(".dll", StringComparison.OrdinalIgnoreCase) && File.Exists(path))
                yield return MetadataReference.CreateFromFile(path);
        }
    }
}

/// <summary>
/// Toy transfer function for <see cref="FrameworkSolverTests"/>: propagates string-literal
/// identities through locals, parameters, and flow captures, and records every value observed at
/// a <c>Effects.Effect(...)</c> call site as an effect. Deliberately has none of
/// <c>CdacProvenanceTransfer</c>'s API-matching logic.
/// </summary>
internal sealed class ToyTransfer : IOperationTransfer<FiniteSetValue<string>, string>
{
    public FiniteSetValue<string> Evaluate(
        IOperation operation,
        FlowState<FiniteSetValue<string>> state,
        IFlowResultSink<FiniteSetValue<string>, string> sink)
    {
        FiniteSetValue<string> result = operation switch
        {
            IConversionOperation conversion => Evaluate(conversion.Operand, state, sink),
            IArgumentOperation argument => Evaluate(argument.Value, state, sink),
            ILiteralOperation literal
                when literal.ConstantValue is { HasValue: true, Value: string text } =>
                    FiniteSetValue<string>.Known(text),
            ILocalReferenceOperation local => state[FlowSlot.ForSymbol(local.Local)],
            IParameterReferenceOperation parameter => state[FlowSlot.ForSymbol(parameter.Parameter)],
            IFlowCaptureReferenceOperation capture => state[FlowSlot.ForCapture(capture.Id)],
            IFlowCaptureOperation capture => Assign(
                FlowSlot.ForCapture(capture.Id),
                Evaluate(capture.Value, state, sink),
                state),
            ISimpleAssignmentOperation assignment => Assign(
                TargetSlot(assignment.Target),
                Evaluate(assignment.Value, state, sink),
                state),
            IVariableDeclaratorOperation declarator when declarator.Initializer is not null => Assign(
                FlowSlot.ForSymbol(declarator.Symbol),
                Evaluate(declarator.Initializer.Value, state, sink),
                state),
            IInvocationOperation { TargetMethod.Name: "Effect" } invocation =>
                RecordEffect(invocation, state, sink),
            _ => EvaluateChildren(operation, state, sink),
        };

        sink.Record(operation, result);
        return result;
    }

    private FiniteSetValue<string> RecordEffect(
        IInvocationOperation invocation,
        FlowState<FiniteSetValue<string>> state,
        IFlowResultSink<FiniteSetValue<string>, string> sink)
    {
        FiniteSetValue<string> argument = Evaluate(invocation.Arguments[0].Value, state, sink);
        foreach (string value in argument.Values)
            sink.AddEffect(value);
        return FiniteSetValue<string>.Bottom;
    }

    private FiniteSetValue<string> EvaluateChildren(
        IOperation operation,
        FlowState<FiniteSetValue<string>> state,
        IFlowResultSink<FiniteSetValue<string>, string> sink)
    {
        foreach (IOperation child in operation.ChildOperations)
            Evaluate(child, state, sink);
        return FiniteSetValue<string>.Bottom;
    }

    private static FiniteSetValue<string> Assign(
        FlowSlot? target,
        FiniteSetValue<string> value,
        FlowState<FiniteSetValue<string>> state)
    {
        if (target.HasValue)
            state[target.Value] = value;
        return value;
    }

    private static FlowSlot? TargetSlot(IOperation operation) =>
        OperationInspector.Unwrap(operation) switch
        {
            ILocalReferenceOperation local => FlowSlot.ForSymbol(local.Local),
            IParameterReferenceOperation parameter => FlowSlot.ForSymbol(parameter.Parameter),
            IFieldReferenceOperation field => FlowSlot.ForSymbol(field.Field),
            IFlowCaptureReferenceOperation capture => FlowSlot.ForCapture(capture.Id),
            IDeclarationExpressionOperation declaration => TargetSlot(declaration.Expression),
            _ => null,
        };
}
