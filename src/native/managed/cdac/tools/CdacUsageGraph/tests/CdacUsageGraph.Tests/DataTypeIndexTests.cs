// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis;
using CdacUsageGraph.Analysis.DataFlow;
using CdacUsageGraph.Compilation;
using CdacUsageGraph.Discovery;
using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.Operations;
using Xunit;

namespace CdacUsageGraph.Tests;

/// <summary>
/// Standalone-compilation unit tests: build a tiny in-memory cDAC-shaped source and assert the
/// discovery phase resolves Data types and native descriptor field names.
/// </summary>
public sealed class DataTypeIndexTests
{
    private const string TypeInfoApiSource = """
        global using Target = Microsoft.Diagnostics.DataContractReader.Target;
        global using TypeInfo = Microsoft.Diagnostics.DataContractReader.Target.TypeInfo;

        namespace Microsoft.Diagnostics.DataContractReader
        {
            public abstract class Target
            {
                public readonly struct FieldInfo
                {
                    public int Offset { get; }
                }

                public readonly struct TypeInfo
                {
                    public System.Collections.Generic.IReadOnlyDictionary<string, FieldInfo> Fields { get; }
                    public uint? Size { get; }
                }

                public abstract TypeInfo GetTypeInfo(string name);
                public abstract bool TryGetTypeInfo(string name, out TypeInfo type);
                public abstract T Read<T>(ulong address);
                public abstract void Write<T>(ulong address, T value);
            }
        }

        namespace Microsoft.Diagnostics.DataContractReader.Contracts
        {
            public interface IManagedTypeSource
            {
                Target.TypeInfo GetTypeInfo(string name);
                bool TryGetTypeInfo(string name, out Target.TypeInfo type);
            }
        }
        """;

    private const string Source = """
        namespace Microsoft.Diagnostics.DataContractReader
        {
            public sealed class CdacTypeAttribute : System.Attribute
            {
                public CdacTypeAttribute(params string[] names) { }
            }
            public sealed class FieldAttribute : System.Attribute
            {
                public FieldAttribute() { }
                public FieldAttribute(params string[] names) { }
                public string[]? Names { get; set; }
            }
        }
        namespace Microsoft.Diagnostics.DataContractReader.Data
        {
            public interface IData<T> { }

            [Microsoft.Diagnostics.DataContractReader.CdacType("Widget")]
            public sealed partial class Widget : IData<Widget>
            {
                [Microsoft.Diagnostics.DataContractReader.Field("m_value")] public int Value { get; }
                [Microsoft.Diagnostics.DataContractReader.Field] public int Count { get; }
            }

            [Microsoft.Diagnostics.DataContractReader.CdacType("NotData")]
            public sealed class NotData { }
        }
        namespace Unrelated
        {
            public interface IData<T> { }

            public sealed class Lookalike : IData<Lookalike> { }
        }
        """;

    private static (CSharpCompilation Compilation, INamedTypeSymbol Widget) BuildWidget()
    {
        CSharpCompilation compilation = CSharpCompilation.Create(
            "DataTypeIndexTest",
            [CSharpSyntaxTree.ParseText(Source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

        INamedTypeSymbol? widget = compilation.GetTypeByMetadataName("Microsoft.Diagnostics.DataContractReader.Data.Widget");
        Assert.NotNull(widget);
        return (compilation, widget!);
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

    [Fact]
    public void DiscoversCdacTypeDataTypes()
    {
        (CSharpCompilation compilation, INamedTypeSymbol widget) = BuildWidget();

        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.True(index.IsDataType(widget));
        Assert.False(index.IsDataType(compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Data.NotData")));
        Assert.False(index.IsDataType(compilation.GetTypeByMetadataName(
            "Unrelated.Lookalike")));
        Assert.Equal(1, index.Count);
    }

    [Theory]
    [InlineData("Value", "m_value")]  // [Field("m_value")] -> native descriptor name
    [InlineData("Count", "Count")]    // [Field] with no explicit name -> property name
    public void ResolvesNativeFieldName(string property, string expectedNative)
    {
        (CSharpCompilation compilation, INamedTypeSymbol widget) = BuildWidget();
        DataTypeIndex index = DataTypeIndex.Build(compilation);

        IPropertySymbol symbol = (IPropertySymbol)widget.GetMembers(property).Single();

        Assert.True(index.TryGetDataType(widget, out DataTypeInfo dataType));
        Assert.Equal(expectedNative, dataType.GetProperty(symbol).NativeName);
    }

    [Fact]
    public void ResolvesCdacNameToDataClass()
    {
        (CSharpCompilation compilation, INamedTypeSymbol widget) = BuildWidget();
        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.True(index.TryGetType("Widget", out DataTypeInfo resolved));
        Assert.Equal(widget, resolved.Symbol, SymbolEqualityComparer.Default);
    }

    [Fact]
    public void UsesFirstCdacNameWhenItDiffersFromClassName()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public enum DataType { WidgetTable }
                public sealed class CdacTypeAttribute : System.Attribute
                {
                    public CdacTypeAttribute(params string[] names) { }
                }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }

                [Microsoft.Diagnostics.DataContractReader.CdacType(
                    nameof(Microsoft.Diagnostics.DataContractReader.DataType.WidgetTable))]
                public sealed class WidgetEntry : IData<WidgetEntry> { }

                [Microsoft.Diagnostics.DataContractReader.CdacType("Managed.Layout")]
                public sealed class ManagedLayout : IData<ManagedLayout> { }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "CdacNameTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol entry = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Data.WidgetEntry")!;

        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.True(index.TryGetDataType(entry, out DataTypeInfo dataType));
        Assert.Equal("WidgetTable", dataType.Name);

        INamedTypeSymbol managedLayout = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Data.ManagedLayout")!;
        Assert.True(index.TryGetDataType(managedLayout, out DataTypeInfo managedDataType));
        Assert.Equal("Managed.Layout", managedDataType.Name);
        Assert.True(index.TryGetType("Managed.Layout", out DataTypeInfo resolvedManagedType));
        Assert.Equal(managedLayout, resolvedManagedType.Symbol, SymbolEqualityComparer.Default);
    }

    [Fact]
    public void DataTypeInfoIncludesInheritedProperties()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class FieldAttribute : System.Attribute { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }

                public class Base
                {
                    [Microsoft.Diagnostics.DataContractReader.Field] public int Value { get; }
                }

                public sealed class Derived : Base, IData<Derived> { }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "InheritedDataPropertyTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol derived = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Data.Derived")!;
        IPropertySymbol baseValue = (IPropertySymbol)compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Data.Base")!
            .GetMembers("Value").Single();

        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.True(index.TryGetDataType(derived, out DataTypeInfo dataType));
        Assert.Equal("Value", dataType.GetProperty(baseValue).NativeName);
    }

    [Fact]
    public void SetterBodyDoesNotMakeAnAutoGetterComputed()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class FieldAttribute : System.Attribute { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }

                public sealed class SetterOnly : IData<SetterOnly>
                {
                    [Microsoft.Diagnostics.DataContractReader.Field]
                    public int Raw { get; private set; }

                    public int Value
                    {
                        get;
                        set { Raw = value; }
                    }
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "SetterBodyPropertyTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol setterOnly = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Data.SetterOnly")!;
        IPropertySymbol value = (IPropertySymbol)setterOnly.GetMembers("Value").Single();

        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.True(index.TryGetDataType(setterOnly, out DataTypeInfo dataType));
        DataPropertyInfo property = dataType.GetProperty(value);
        Assert.Equal(DataPropertyKind.DirectField, property.Kind);
        Assert.Empty(property.ExpansionMembers);
    }

    [Fact]
    public void OnInitAssignmentDefinesPropertyProvenanceWithoutNullabilityAttribute()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class CdacTypeAttribute : System.Attribute
                {
                    public CdacTypeAttribute(params string[] names) { }
                }
                public sealed class FieldAttribute : System.Attribute { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }

                [Microsoft.Diagnostics.DataContractReader.CdacType("Widget")]
                public sealed partial class Widget : IData<Widget>
                {
                    [Microsoft.Diagnostics.DataContractReader.Field]
                    public int Raw { get; }

                    public int Derived { get; private set; }
                    public int CompoundDerived { get; private set; }

                    partial void OnInit();
                }

                public sealed partial class Widget
                {
                    partial void OnInit()
                    {
                        Derived = Raw;
                        CompoundDerived += Raw;
                    }
                }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Contracts
            {
                public sealed class TestContract
                {
                    public int Read(Microsoft.Diagnostics.DataContractReader.Data.Widget widget)
                        => widget.Derived + widget.CompoundDerived;
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "OnInitPropertyTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol impl = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Contracts.TestContract")!;
        DataTypeIndex index = DataTypeIndex.Build(compilation);
        UsageGraph graph = new UsageWalker(compilation, index).Walk(
            [new ContractRegistration("ITest", "c1", impl)], "");
        IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>> fields = graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Widget")];

        Assert.Contains("Raw", fields.Keys);
        Assert.DoesNotContain("Derived", fields.Keys);
        Assert.DoesNotContain("CompoundDerived", fields.Keys);
    }

    [Fact]
    public void CompoundAssignmentRightOperandIsReadOnly()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class CdacTypeAttribute : System.Attribute
                {
                    public CdacTypeAttribute(params string[] names) { }
                }
                public sealed class FieldAttribute : System.Attribute { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }

                [Microsoft.Diagnostics.DataContractReader.CdacType("Widget")]
                public sealed class Widget : IData<Widget>
                {
                    [Microsoft.Diagnostics.DataContractReader.Field]
                    public int Value { get; }
                }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Contracts
            {
                public sealed class TestContract
                {
                    public int Read(Microsoft.Diagnostics.DataContractReader.Data.Widget widget)
                    {
                        int value = 1;
                        value += widget.Value;
                        return value;
                    }
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "CompoundAssignmentReadTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol impl = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Contracts.TestContract")!;
        DataTypeIndex index = DataTypeIndex.Build(compilation);
        UsageGraph graph = new UsageWalker(compilation, index).Walk(
            [new ContractRegistration("ITest", "c1", impl)], "");
        IReadOnlyCollection<UsageKind> kinds = graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Widget")]["Value"];

        Assert.Equal([UsageKind.Read], kinds);
    }

    [Fact]
    public void ResolvesGetTypeInfoEnumArgumentWithoutDataTypeNameHeuristic()
    {
        const string source = """
            namespace Example
            {
                public enum LayoutKind { Widget }

                public sealed class Target
                {
                    public void GetTypeInfo(LayoutKind type) { }
                    public void GetTypeInfo(string name) { }
                }

                public sealed class User
                {
                    public void Read(Target target) => target.GetTypeInfo(LayoutKind.Widget);
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "EnumGetTypeInfoTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        IMethodSymbol read = compilation.GetTypeByMetadataName("Example.User")!
            .GetMembers("Read").OfType<IMethodSymbol>().Single();
        IOperation operation = compilation.GetSemanticModel(read.DeclaringSyntaxReferences[0].SyntaxTree)
            .GetOperation(read.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation invocation = operation.DescendantsAndSelf()
            .OfType<IInvocationOperation>().Single();

        Assert.Equal("Widget", OperationInspector.InlineGetTypeInfoName(invocation));
    }

    [Fact]
    public void TypeInfoDfaRecordsDictionaryFieldProbes()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    public void Read(Target target)
                    {
                        TypeInfo type = target.GetTypeInfo("Widget");
                        _ = type.Fields.ContainsKey("Contains");
                        if (type.Fields.TryGetValue("Try", out Target.FieldInfo field))
                            _ = target.Read<int>(100UL + (ulong)field.Offset);
                    }
                }
            }
            """;
        (TypeInfoFlowResult result, _, _) =
            AnalyzeTypeInfoFlow(source, "Example.User", "Read");

        Assert.Contains(
            new FieldAccessEffect(
                new FieldIdentity("Widget", "Contains"),
                UsageKind.OffsetLookup),
            result.Effects);
        Assert.Contains(
            new FieldAccessEffect(
                new FieldIdentity("Widget", "Try"),
                UsageKind.OffsetLookup),
            result.Effects);
        Assert.Contains(
            new FieldAccessEffect(
                new FieldIdentity("Widget", "Try"),
                UsageKind.Read),
            result.Effects);
    }

    [Fact]
    public void TypeInfoDfaTracksSequentialReassignmentAtEachUse()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private static void Use(TypeInfo type) { }

                    public void Read(Target target)
                    {
                        TypeInfo type = target.GetTypeInfo("First");
                        Use(type);
                        type = target.GetTypeInfo("Second");
                        Use(type);
                    }
                }
            }
            """;
        (TypeInfoFlowResult result, IMethodSymbol method, SemanticModel model) =
            AnalyzeTypeInfoFlow(source, "Example.User", "Read");
        IOperation body = model.GetOperation(method.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation[] uses = body.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Where(invocation => invocation.TargetMethod.Name == "Use")
            .ToArray();

        Assert.Equal(["First"], result.GetValue(uses[0].Arguments[0].Value).Names);
        Assert.Equal(["Second"], result.GetValue(uses[1].Arguments[0].Value).Names);
    }

    [Fact]
    public void TypeInfoDfaJoinsBranchValues()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private static void Use(TypeInfo type) { }

                    public void Read(Target target, bool condition)
                    {
                        TypeInfo type;
                        if (condition)
                            type = target.GetTypeInfo("First");
                        else
                            type = target.GetTypeInfo("Second");
                        Use(type);
                    }
                }
            }
            """;
        (TypeInfoFlowResult result, IMethodSymbol method, SemanticModel model) =
            AnalyzeTypeInfoFlow(source, "Example.User", "Read");
        IOperation body = model.GetOperation(method.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = body.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(
            ["First", "Second"],
            result.GetValue(use.Arguments[0].Value).Names.OrderBy(name => name, StringComparer.Ordinal));
    }

    [Fact]
    public void TypeInfoDfaConvergesAcrossLoopBackEdge()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private static void Use(TypeInfo type) { }

                    public void Read(Target target, bool repeat)
                    {
                        TypeInfo type = target.GetTypeInfo("First");
                        while (repeat)
                        {
                            Use(type);
                            type = target.GetTypeInfo("Second");
                        }
                    }
                }
            }
            """;
        (TypeInfoFlowResult result, IMethodSymbol method, SemanticModel model) =
            AnalyzeTypeInfoFlow(source, "Example.User", "Read");
        IOperation body = model.GetOperation(method.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = body.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(
            ["First", "Second"],
            result.GetValue(use.Arguments[0].Value).Names.OrderBy(name => name, StringComparer.Ordinal));
    }

    [Fact]
    public void TypeInfoDfaResultsRemainStableWhenAnalyzerIsReused()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private static void Use(TypeInfo type) { }
                    public void First(Target target) => Use(target.GetTypeInfo("First"));
                    public void Second(Target target) => Use(target.GetTypeInfo("Second"));
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        INamedTypeSymbol user = compilation.GetTypeByMetadataName("Example.User")!;
        IMethodSymbol firstMethod = user.GetMembers("First").OfType<IMethodSymbol>().Single();
        IMethodSymbol secondMethod = user.GetMembers("Second").OfType<IMethodSymbol>().Single();
        SemanticModel model = compilation.GetSemanticModel(firstMethod.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation firstRoot = model.GetOperation(firstMethod.DeclaringSyntaxReferences[0].GetSyntax())!;
        IOperation secondRoot = model.GetOperation(secondMethod.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation firstUse = firstRoot.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");
        TypeInfoDataFlowAnalysis analysis = new(
            CdacApiSymbols.Build(new CdacAnalysisWorkspace(compilation, [compilation])));

        TypeInfoFlowResult firstResult = analysis.Analyze(firstRoot);
        _ = analysis.Analyze(secondRoot);

        Assert.Equal(["First"], firstResult.GetValue(firstUse.Arguments[0].Value).Names);
    }

    [Fact]
    public void TypeInfoDfaTracksTryGetTypeInfoOutValue()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private static void Use(TypeInfo type) { }

                    public void Read(Target target)
                    {
                        target.TryGetTypeInfo("First", out TypeInfo type);
                        Use(type);
                    }
                }
            }
            """;
        (TypeInfoFlowResult result, IMethodSymbol method, SemanticModel model) =
            AnalyzeTypeInfoFlow(source, "Example.User", "Read");
        IOperation body = model.GetOperation(method.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = body.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], result.GetValue(use.Arguments[0].Value).Names);
    }

    [Fact]
    public void TypeInfoDfaRejectsLookalikeProducerMethod()
    {
        const string source = """
            namespace Example
            {
                public sealed class FakeTarget
                {
                    public TypeInfo GetTypeInfo(string name) => default;
                }

                public sealed class User
                {
                    private static void Use(TypeInfo type) { }

                    public void Read(FakeTarget target)
                    {
                        TypeInfo type = target.GetTypeInfo("First");
                        Use(type);
                    }
                }
            }
            """;
        (TypeInfoFlowResult result, IMethodSymbol method, SemanticModel model) =
            AnalyzeTypeInfoFlow(source, "Example.User", "Read");
        IOperation body = model.GetOperation(method.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = body.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Empty(result.GetValue(use.Arguments[0].Value).Names);
    }

    [Fact]
    public void TypeInfoDfaRecordsSizeEffect()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    public uint? Read(Target target)
                        => target.GetTypeInfo("Widget").Size;
                }
            }
            """;
        (TypeInfoFlowResult result, _, _) =
            AnalyzeTypeInfoFlow(source, "Example.User", "Read");

        Assert.Contains(
            new FieldAccessEffect(new FieldIdentity("Widget", "Size"), UsageKind.Read),
            result.Effects);
    }

    [Fact]
    public void TypeInfoDfaPropagatesHelperReturnValue()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private static TypeInfo GetLayout(Target target)
                        => target.GetTypeInfo("First");

                    private static void Use(TypeInfo type) { }

                    public void Read(Target target)
                    {
                        TypeInfo type = GetLayout(target);
                        Use(type);
                    }
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        CdacAnalysisWorkspace workspace = new(compilation, [compilation]);
        DataFlowTypeInfoResolver resolver = new(workspace);
        IMethodSymbol read = compilation.GetTypeByMetadataName("Example.User")!
            .GetMembers("Read").OfType<IMethodSymbol>().Single();
        SemanticModel model = compilation.GetSemanticModel(
            read.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(read.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = root.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], resolver.GetTypeInfoDataNames(use.Arguments[0].Value));
    }

    [Fact]
    public void TypeInfoDfaPropagatesHelperOutValue()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private static void GetLayout(Target target, out TypeInfo type)
                        => type = target.GetTypeInfo("First");

                    private static void Use(TypeInfo type) { }

                    public void Read(Target target)
                    {
                        GetLayout(target, out TypeInfo type);
                        Use(type);
                    }
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        CdacAnalysisWorkspace workspace = new(compilation, [compilation]);
        DataFlowTypeInfoResolver resolver = new(workspace);
        IMethodSymbol read = compilation.GetTypeByMetadataName("Example.User")!
            .GetMembers("Read").OfType<IMethodSymbol>().Single();
        SemanticModel model = compilation.GetSemanticModel(
            read.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(read.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = root.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], resolver.GetTypeInfoDataNames(use.Arguments[0].Value));
    }

    [Fact]
    public void TypeInfoDfaPropagatesIntoPartialMethodImplementation()
    {
        const string source = """
            namespace Example
            {
                public sealed partial class User
                {
                    partial void Forward(TypeInfo type);
                    private static void Use(TypeInfo type) { }

                    public void Read(Target target)
                        => Forward(target.GetTypeInfo("First"));
                }

                public sealed partial class User
                {
                    partial void Forward(TypeInfo type)
                        => Use(type);
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        CdacAnalysisWorkspace workspace = new(compilation, [compilation]);
        DataFlowTypeInfoResolver resolver = new(workspace);
        IMethodSymbol forward = compilation.GetTypeByMetadataName("Example.User")!
            .GetMembers("Forward")
            .OfType<IMethodSymbol>()
            .Single(method => method.PartialImplementationPart is not null)
            .PartialImplementationPart!;
        SemanticModel model = compilation.GetSemanticModel(
            forward.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(forward.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = root.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], resolver.GetTypeInfoDataNames(use.Arguments[0].Value));
    }

    [Fact]
    public void TypeInfoDfaPropagatesIntoMostDerivedInterfaceOverride()
    {
        const string source = """
            namespace Example
            {
                public interface IForwarder
                {
                    void Forward(TypeInfo type);
                }

                public class Base : IForwarder
                {
                    public virtual void Forward(TypeInfo type) { }
                }

                public sealed class Derived : Base
                {
                    private static void Use(TypeInfo type) { }
                    public override void Forward(TypeInfo type) => Use(type);
                }

                public sealed class User
                {
                    public void Read(Target target)
                    {
                        IForwarder forwarder = new Derived();
                        forwarder.Forward(target.GetTypeInfo("First"));
                    }
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        DataFlowTypeInfoResolver resolver = new(
            new CdacAnalysisWorkspace(compilation, [compilation]));
        IMethodSymbol forward = compilation.GetTypeByMetadataName("Example.Derived")!
            .GetMembers("Forward")
            .OfType<IMethodSymbol>()
            .Single();
        SemanticModel model = compilation.GetSemanticModel(
            forward.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(
            forward.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = root.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], resolver.GetTypeInfoDataNames(use.Arguments[0].Value));
    }

    [Fact]
    public void TypeInfoDfaPropagatesReducedExtensionReceiver()
    {
        const string source = """
            namespace Example
            {
                public static class Extensions
                {
                    private static void Use(TypeInfo type) { }
                    public static void Forward(this TypeInfo type) => Use(type);
                }

                public sealed class User
                {
                    public void Read(Target target)
                        => target.GetTypeInfo("First").Forward();
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        DataFlowTypeInfoResolver resolver = new(
            new CdacAnalysisWorkspace(compilation, [compilation]));
        IMethodSymbol forward = compilation.GetTypeByMetadataName("Example.Extensions")!
            .GetMembers("Forward")
            .OfType<IMethodSymbol>()
            .Single();
        SemanticModel model = compilation.GetSemanticModel(
            forward.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(
            forward.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = root.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], resolver.GetTypeInfoDataNames(use.Arguments[0].Value));
    }

    [Fact]
    public void TypeInfoDfaPropagatesOutValueStoredInField()
    {
        const string source = """
            namespace Example
            {
                public sealed class User
                {
                    private TypeInfo _type;
                    private static void Use(TypeInfo type) { }

                    public void Initialize(Target target)
                        => target.TryGetTypeInfo("First", out _type);

                    public void Read()
                        => Use(_type);
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        CdacAnalysisWorkspace workspace = new(compilation, [compilation]);
        DataFlowTypeInfoResolver resolver = new(workspace);
        IMethodSymbol read = compilation.GetTypeByMetadataName("Example.User")!
            .GetMembers("Read")
            .OfType<IMethodSymbol>()
            .Single();
        SemanticModel model = compilation.GetSemanticModel(
            read.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(read.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = root.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], resolver.GetTypeInfoDataNames(use.Arguments[0].Value));
    }

    [Fact]
    public void TypeInfoDfaFollowsEnumForwardingWrapperWithoutLegacyFallback()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public enum DataType { First }

                internal static class DataTypeNames
                {
                    public static string ToName(this DataType type)
                        => type switch
                        {
                            DataType.First => nameof(DataType.First),
                            _ => throw new System.ArgumentOutOfRangeException(nameof(type)),
                        };
                }

                public static class DataTypeTargetExtensions
                {
                    public static Target.TypeInfo GetTypeInfo(this Target target, DataType type)
                        => target.GetTypeInfo(type.ToName());
                }
            }

            namespace Example
            {
                using Microsoft.Diagnostics.DataContractReader;

                public sealed class User
                {
                    private static void Use(TypeInfo type) { }

                    public void Read(Target target)
                    {
                        TypeInfo type = target.GetTypeInfo(DataType.First);
                        Use(type);
                    }
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        CdacAnalysisWorkspace workspace = new(compilation, [compilation]);
        DataFlowTypeInfoResolver resolver = new(workspace);
        IMethodSymbol read = compilation.GetTypeByMetadataName("Example.User")!
            .GetMembers("Read").OfType<IMethodSymbol>().Single();
        SemanticModel model = compilation.GetSemanticModel(
            read.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(read.DeclaringSyntaxReferences[0].GetSyntax())!;
        IInvocationOperation use = root.DescendantsAndSelf().OfType<IInvocationOperation>()
            .Single(invocation => invocation.TargetMethod.Name == "Use");

        Assert.Equal(["First"], resolver.GetTypeInfoDataNames(use.Arguments[0].Value));
    }

    private static (TypeInfoFlowResult Result, IMethodSymbol Method, SemanticModel Model) AnalyzeTypeInfoFlow(
        string source,
        string typeName,
        string methodName)
    {
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        IMethodSymbol method = compilation.GetTypeByMetadataName(typeName)!
            .GetMembers(methodName)
            .OfType<IMethodSymbol>()
            .Single();
        SemanticModel model = compilation.GetSemanticModel(
            method.DeclaringSyntaxReferences[0].SyntaxTree);
        IOperation root = model.GetOperation(method.DeclaringSyntaxReferences[0].GetSyntax())!;

        CdacApiSymbols apiSymbols = CdacApiSymbols.Build(
            new CdacAnalysisWorkspace(compilation, [compilation]));
        return (new TypeInfoDataFlowAnalysis(apiSymbols).Analyze(root), method, model);
    }

    private static CSharpCompilation CreateTypeInfoFlowCompilation(string source) =>
        CSharpCompilation.Create(
            "TypeInfoDfaTest",
            [CSharpSyntaxTree.ParseText(TypeInfoApiSource), CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

    [Fact]
    public void InterfaceComputedPropertyUsesSameProvenanceAsDirectRead()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class CdacTypeAttribute : System.Attribute
                {
                    public CdacTypeAttribute(params string[] names) { }
                }
                public sealed class FieldAttribute : System.Attribute { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }
                public interface IValue { int Value { get; } }

                [Microsoft.Diagnostics.DataContractReader.CdacType("Computed")]
                public sealed class Computed : IData<Computed>, IValue
                {
                    [Microsoft.Diagnostics.DataContractReader.Field] public int Raw { get; }
                    public int Value => Raw;
                }

                [Microsoft.Diagnostics.DataContractReader.CdacType("Direct")]
                public sealed class Direct : IData<Direct>, IValue
                {
                    [Microsoft.Diagnostics.DataContractReader.Field] public int Value { get; }
                }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Contracts
            {
                public sealed class TestContract
                {
                    public int Read(Microsoft.Diagnostics.DataContractReader.Data.IValue value)
                        => value.Value;
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "InterfaceComputedPropertyTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol impl = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Contracts.TestContract")!;
        DataTypeIndex index = DataTypeIndex.Build(compilation);
        UsageGraph graph = new UsageWalker(compilation, index).Walk(
            [new ContractRegistration("ITest", "c1", impl)], "");

        Assert.Contains("Raw", graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Computed")].Keys);
        Assert.DoesNotContain("Value", graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Computed")].Keys);
        Assert.Contains("Value", graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Direct")].Keys);
    }

    [Fact]
    public void WalksGenericMemberForEachTypeSubstitution()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class CdacTypeAttribute : System.Attribute
                {
                    public CdacTypeAttribute(params string[] names) { }
                }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }
                [Microsoft.Diagnostics.DataContractReader.CdacType("First")]
                public sealed class First : IData<First> { }
                [Microsoft.Diagnostics.DataContractReader.CdacType("Second")]
                public sealed class Second : IData<Second> { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Contracts
            {
                using Microsoft.Diagnostics.DataContractReader.Data;

                public sealed class ProcessedData
                {
                    public T GetOrAdd<T>(int address) => default!;
                }

                public sealed class Helper<T>
                {
                    private readonly ProcessedData _data;
                    public Helper(ProcessedData data) => _data = data;
                    public T Load() => _data.GetOrAdd<T>(0);
                }

                public sealed class TestContract
                {
                    private readonly ProcessedData _data = new();
                    public void ReadBoth()
                    {
                        _ = new Helper<First>(_data).Load();
                        _ = new Helper<Second>(_data).Load();
                    }
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "GenericSubstitutionTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol impl = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Contracts.TestContract")!;
        DataTypeIndex index = DataTypeIndex.Build(compilation);
        UsageGraph graph = new UsageWalker(
            compilation, index).Walk(
                [new ContractRegistration("ITest", "c1", impl)], "");
        ContractLabel label = new("ITest", "c1");
        HashSet<string> used = graph.FieldUsage.Keys
            .Where(k => k.Label == label)
            .Select(k => k.DataType)
            .ToHashSet(StringComparer.Ordinal);

        Assert.Contains("Data.First", used);
        Assert.Contains("Data.Second", used);
    }

    [Fact]
    public void UsageWalkerKeepsDataFlowEffectsInCallingContractContext()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class CdacTypeAttribute : System.Attribute
                {
                    public CdacTypeAttribute(params string[] names) { }
                }
                public sealed class FieldAttribute : System.Attribute { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }

                [Microsoft.Diagnostics.DataContractReader.CdacType("First")]
                public sealed class First : IData<First>
                {
                    [Microsoft.Diagnostics.DataContractReader.Field]
                    public int Value { get; }
                }

                [Microsoft.Diagnostics.DataContractReader.CdacType("Second")]
                public sealed class Second : IData<Second>
                {
                    [Microsoft.Diagnostics.DataContractReader.Field]
                    public int Value { get; }
                }
            }
            namespace Example
            {
                public static class Helper
                {
                    public static int Read(Target target, TypeInfo type)
                        => target.Read<int>(100UL + (ulong)type.Fields["Value"].Offset);
                }

                public sealed class FirstContract
                {
                    public int Read(Target target)
                        => Helper.Read(target, target.GetTypeInfo("First"));
                }

                public sealed class SecondContract
                {
                    public int Read(Target target)
                        => Helper.Read(target, target.GetTypeInfo("Second"));
                }
            }
            """;
        CSharpCompilation compilation = CreateTypeInfoFlowCompilation(source);
        CdacAnalysisWorkspace workspace = new(compilation, [compilation]);
        DataTypeIndex index = DataTypeIndex.Build(compilation);
        UsageGraph graph = new UsageWalker(
            compilation,
            index,
            new DataFlowTypeInfoResolver(workspace)).Walk(
                [
                    new ContractRegistration(
                        "IFirst",
                        "c1",
                        compilation.GetTypeByMetadataName("Example.FirstContract")!),
                    new ContractRegistration(
                        "ISecond",
                        "c1",
                        compilation.GetTypeByMetadataName("Example.SecondContract")!),
                ],
                "");

        Assert.True(graph.FieldUsage.ContainsKey(
            (new ContractLabel("IFirst", "c1"), "Data.First")));
        Assert.False(graph.FieldUsage.ContainsKey(
            (new ContractLabel("IFirst", "c1"), "Data.Second")));
        Assert.True(graph.FieldUsage.ContainsKey(
            (new ContractLabel("ISecond", "c1"), "Data.Second")));
        Assert.False(graph.FieldUsage.ContainsKey(
            (new ContractLabel("ISecond", "c1"), "Data.First")));
    }

    [Fact]
    public void ContractEntryPointUsesMostDerivedInterfaceOverride()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class CdacTypeAttribute : System.Attribute
                {
                    public CdacTypeAttribute(params string[] names) { }
                }
                public sealed class FieldAttribute : System.Attribute { }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Data
            {
                public interface IData<T> { }

                [Microsoft.Diagnostics.DataContractReader.CdacType("BaseData")]
                public sealed class BaseData : IData<BaseData>
                {
                    [Microsoft.Diagnostics.DataContractReader.Field]
                    public int Value { get; }
                }

                [Microsoft.Diagnostics.DataContractReader.CdacType("DerivedData")]
                public sealed class DerivedData : IData<DerivedData>
                {
                    [Microsoft.Diagnostics.DataContractReader.Field]
                    public int Value { get; }
                }
            }
            namespace Example
            {
                public interface ITest
                {
                    int Read(
                        Microsoft.Diagnostics.DataContractReader.Data.BaseData baseData,
                        Microsoft.Diagnostics.DataContractReader.Data.DerivedData derivedData);
                }

                public class Base : ITest
                {
                    public virtual int Read(
                        Microsoft.Diagnostics.DataContractReader.Data.BaseData baseData,
                        Microsoft.Diagnostics.DataContractReader.Data.DerivedData derivedData)
                        => baseData.Value;
                }

                public sealed class Derived : Base
                {
                    public override int Read(
                        Microsoft.Diagnostics.DataContractReader.Data.BaseData baseData,
                        Microsoft.Diagnostics.DataContractReader.Data.DerivedData derivedData)
                        => derivedData.Value;
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "InterfaceOverrideReachabilityTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol @interface = compilation.GetTypeByMetadataName("Example.ITest")!;
        INamedTypeSymbol implementation = compilation.GetTypeByMetadataName("Example.Derived")!;
        UsageGraph graph = new UsageWalker(
            compilation,
            DataTypeIndex.Build(compilation)).Walk(
                [new ContractRegistration(
                    "ITest",
                    "c1",
                    implementation,
                    @interface,
                    Constructor: null)],
                "");
        ContractLabel label = new("ITest", "c1");

        Assert.Contains(
            "Value",
            graph.FieldUsage[(label, "Data.DerivedData")].Keys);
        Assert.False(graph.FieldUsage.ContainsKey((label, "Data.BaseData")));
        Assert.Contains(
            graph.ReachableMethods[label],
            method => method.Contains("Example.Derived.Read", StringComparison.Ordinal));
        Assert.DoesNotContain(
            graph.ReachableMethods[label],
            method => method.Contains("Example.Base.Read", StringComparison.Ordinal));
    }

    [Fact]
    public void DiscoversRegistrationsOutsideMethodNamedRegisterAndIgnoresNestedHelpers()
    {
        const string source = """
            namespace Microsoft.Diagnostics.DataContractReader
            {
                public sealed class ContractRegistry
                {
                    public void Register<T>(string version, System.Func<object, T> factory) { }
                }
            }
            namespace Microsoft.Diagnostics.DataContractReader.Contracts
            {
                public interface IContract { }
                public interface ITest : IContract { }
                public sealed class Helper { }
                public sealed class Impl : ITest
                {
                    public Impl(Helper helper) { }
                }
                public static class CoreCLRContracts
                {
                    public static void Configure(
                        Microsoft.Diagnostics.DataContractReader.ContractRegistry registry)
                    {
                        registry.Register<ITest>("c1", _ => new Impl(new Helper()));
                        new UnrelatedRegistry().Register<ITest>("c2", _ => new Impl(new Helper()));
                        registry.Register<ITest>("c3", _ => new NonContractImplementation());
                    }
                }

                public sealed class NonContractImplementation { }
                public sealed class UnrelatedRegistry
                {
                    public void Register<T>(string version, System.Func<object, T> factory) { }
                }
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "RegistrationDiscoveryTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

        IReadOnlyList<ContractRegistration> registrations =
            ContractRegistrationParser.Parse(compilation);

        ContractRegistration registration = Assert.Single(registrations);
        Assert.Equal("ITest", registration.Contract);
        Assert.Equal("c1", registration.Version);
        Assert.Equal("Impl", registration.Impl.Name);
        Assert.Equal("ITest", registration.Interface!.Name);
        Assert.Equal("Impl", registration.Constructor!.ContainingType.Name);
    }
}
