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
            compilation, index, TypeInfoCorrelator.Build(compilation)).Walk(
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
    }
}
