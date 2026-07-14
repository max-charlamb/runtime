// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis;
using CdacUsageGraph.Discovery;
using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Xunit;

namespace CdacUsageGraph.Tests;

/// <summary>
/// Standalone-compilation unit tests: build a tiny in-memory cDAC-shaped source and assert the
/// discovery phase resolves Data types and native descriptor field names.
/// </summary>
public sealed class DataTypeIndexTests
{
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
        }
        """;

    private static (Microsoft.CodeAnalysis.Compilation Compilation, INamedTypeSymbol Widget) BuildWidget()
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
        (Microsoft.CodeAnalysis.Compilation compilation, INamedTypeSymbol widget) = BuildWidget();

        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.True(index.IsDataType(widget));
        Assert.Equal(1, index.Count);
    }

    [Theory]
    [InlineData("Value", "m_value")]  // [Field("m_value")] -> native descriptor name
    [InlineData("Count", "Count")]    // [Field] with no explicit name -> property name
    public void ResolvesNativeFieldName(string property, string expectedNative)
    {
        (Microsoft.CodeAnalysis.Compilation compilation, INamedTypeSymbol widget) = BuildWidget();
        DataTypeIndex index = DataTypeIndex.Build(compilation);

        IPropertySymbol symbol = (IPropertySymbol)widget.GetMembers(property).Single();

        Assert.Equal(expectedNative, index.NativeName(symbol));
    }

    [Fact]
    public void ResolvesDescriptorNameToDataClass()
    {
        (Microsoft.CodeAnalysis.Compilation compilation, INamedTypeSymbol widget) = BuildWidget();
        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.True(index.TryGetType("Widget", out INamedTypeSymbol resolved));
        Assert.Equal(widget, resolved, SymbolEqualityComparer.Default);
    }

    [Fact]
    public void UsesNativeDescriptorNameWhenItDiffersFromClassName()
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
            }
            """;
        CSharpCompilation compilation = CSharpCompilation.Create(
            "DescriptorNameTest",
            [CSharpSyntaxTree.ParseText(source)],
            RuntimeReferences(),
            new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));
        INamedTypeSymbol entry = compilation.GetTypeByMetadataName(
            "Microsoft.Diagnostics.DataContractReader.Data.WidgetEntry")!;

        DataTypeIndex index = DataTypeIndex.Build(compilation);

        Assert.Equal("WidgetTable", index.DescriptorName(entry));
    }

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
        TypeInfoCorrelator correlator = TypeInfoCorrelator.Build(compilation);

        UsageGraph graph = new UsageWalker(compilation, index, correlator).Walk(
            [new ContractRegistration("ITest", "c1", impl)], "");

        Assert.Contains("Raw", graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Computed")].Keys);
        Assert.DoesNotContain("Value", graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Computed")].Keys);
        Assert.Contains("Value", graph.FieldUsage[
            (new ContractLabel("ITest", "c1"), "Data.Direct")].Keys);
    }
}
