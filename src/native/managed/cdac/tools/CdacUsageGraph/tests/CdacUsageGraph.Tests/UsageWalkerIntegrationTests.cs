// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis;
using CdacUsageGraph.Compilation;
using CdacUsageGraph.Discovery;
using CdacUsageGraph.Model;
using Xunit;

namespace CdacUsageGraph.Tests;
/// <summary>
/// End-to-end test over the real in-repo cDAC source. Guards the whole pipeline against known
/// invariants (a form of the "verify against known facts" baseline pattern). Skipped when the
/// cDAC source cannot be located (e.g. running outside the repo).
/// </summary>
public sealed class UsageWalkerIntegrationTests
{
    private static (UsageGraph Graph, string Root)? BuildRealGraph()
    {
        DirectoryInfo? root = Locator.FindCdacRoot();
        if (root is null)
            return null;

        Microsoft.CodeAnalysis.Compilation compilation = new CdacCompilationLoader().Load(root.FullName);
        DataTypeIndex index = DataTypeIndex.Build(compilation);
        IReadOnlyList<ContractRegistration> registrations = ContractRegistrationParser.Parse(compilation);
        TypeInfoCorrelator correlator = TypeInfoCorrelator.Build(compilation);
        UsageGraph graph = new UsageWalker(compilation, index, correlator).Walk(registrations, root.FullName);
        return (graph, root.FullName);
    }

    [Fact]
    public void ThreadContractUsesThreadDataAndObjectContract()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        Assert.True(graph.Registrations.Count >= 39, $"expected >=39 registrations, got {graph.Registrations.Count}");
        Assert.True(graph.DataTypeCount > 100, $"expected >100 Data types, got {graph.DataTypeCount}");

        // IThread c1 directly reads Data.Thread ...
        HashSet<string> threadTypes = DataTypesUsed(graph, new ContractLabel("IThread", "c1"));
        Assert.Contains("Data.Thread", threadTypes);

        // ... and depends on the Object contract (not on Object's descriptors directly).
        Assert.True(graph.ContractsUsed.TryGetValue(new ContractLabel("IThread", "c1"), out IReadOnlyCollection<string>? threadContracts));
        Assert.Contains("Object", threadContracts!);
    }

    [Fact]
    public void ResolvesNativeDescriptorFieldNames()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        // Exception.Message is [Field("_message")] -> the native descriptor name is emitted.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("IException", "c1"), "Data.Exception"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? exceptionFields));
        Assert.Contains("_message", exceptionFields!.Keys);
    }

    [Fact]
    public void ResolvesGenericBaseAndStaticAbstractDispatch()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        // PrecodeStubs c3 reaches Data types only via a generic base + static-abstract dispatch.
        HashSet<string> precodeTypes = DataTypesUsed(graph, new ContractLabel("IPrecodeStubs", "c3"));
        Assert.Contains("Data.InterpMethod", precodeTypes);
    }

    [Fact]
    public void ResolvesFieldReadsReachedThroughFieldInitializerHelper()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        // StressLog_1's SmallStressMessageReader is constructed in a field initializer and reads
        // Data.StressMsg fields; walking initializers is what surfaces these.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("IStressLog", "c1"), "Data.StressMsg"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? stressMsgFields));
        Assert.Contains("Header", stressMsgFields!.Keys);

        // StressMsgHeader is used only via GetTypeInfo(DataType.StressMsgHeader).Size.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("IStressLog", "c1"), "Data.StressMsgHeader"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? headerFields));
        Assert.Contains("Size", headerFields!.Keys);
    }

    [Fact]
    public void ResolvesFieldReadsThroughSharedDataInterface()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        // ExecutionManager reads exception-clause fields through the IExceptionClauseData interface
        // on a local that may hold either R2RExceptionClause or EEExceptionClause. Both Data types'
        // [Field] members should be attributed -- in particular R2RExceptionClause, which is never
        // referenced by a concrete-typed read.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("IExecutionManager", "c1"), "Data.R2RExceptionClause"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? r2rFields));
        Assert.Contains("Flags", r2rFields!.Keys);
        Assert.Contains("ClassToken", r2rFields!.Keys); // a [Field] on R2R (computed on EE, so EE is not credited it)

        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("IExecutionManager", "c1"), "Data.EEExceptionClause"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? eeFields));
        Assert.Contains("Flags", eeFields!.Keys);

        // Computed (non-[Field]) interface members map to no descriptor field on either type.
        Assert.DoesNotContain("FilterOffset", r2rFields!.Keys);
        Assert.DoesNotContain("FilterOffset", eeFields!.Keys);
    }

    [Fact]
    public void ComputedConveniencePropertiesResolveToUnderlyingFields()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        // IThread reads TLSIndex.IndexOffset/IsAllocated, which are computed (=> TLSIndexRawIndex & ...).
        // The tool records the actual underlying [Field] (TLSIndexRawIndex), not the derived names.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("IThread", "c1"), "Data.TLSIndex"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? tlsFields));
        Assert.Contains("TLSIndexRawIndex", tlsFields!.Keys);
        Assert.DoesNotContain("IndexOffset", tlsFields!.Keys);
        Assert.DoesNotContain("IsAllocated", tlsFields!.Keys);

        // OnInit-populated auto-properties (read by name, no computed getter) are actual fields and
        // are kept -- e.g. Thread.ThreadHandle / Thread.RuntimeThreadLocals.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("IThread", "c1"), "Data.Thread"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? threadFields));
        Assert.Contains("ThreadHandle", threadFields!.Keys);
        Assert.Contains("RuntimeThreadLocals", threadFields!.Keys);
    }

    [Theory]
    [InlineData("Data.EETypeHashTable")]
    [InlineData("Data.InstMethodHashTable")]
    public void ResolvesFieldsReadThroughReusableTypeInfoHelper(string dataType)
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        // Both hash Data types populate a derived Entries collection by passing their TypeInfo
        // through OnInit -> DacEnumerableHash's constructor parameter -> _type field. The helper
        // reads the actual descriptor fields through Read*Field/TypeInfo.Fields, so those fields
        // should be attributed to each concrete hash type rather than the aggregate Entries name.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("ILoader", "c1"), dataType),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? fields));
        Assert.Contains("Buckets", fields!.Keys);
        Assert.Contains("Count", fields.Keys);
        Assert.Contains("VolatileEntryNextEntry", fields.Keys);
        Assert.Contains("VolatileEntryValue", fields.Keys);
        Assert.DoesNotContain("Entries", fields.Keys);
    }

    [Fact]
    public void AttributesDynamicILFieldsAcrossSHashContractBoundary()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;
        const string dataType = "Data.DynamicILBlobTable";

        // Loader owns the entry traits/direct result read. DynamicILBlobEntry is an adapter C#
        // class for the native DynamicILBlobTable descriptor, so the native descriptor name is
        // emitted and the constructor-derived HashTable aggregate is not.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("ILoader", "c1"), dataType),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? loaderFields));
        Assert.Contains("EntryIL", loaderFields!.Keys);
        Assert.Contains("EntryMethodToken", loaderFields.Keys);
        Assert.DoesNotContain("HashTable", loaderFields.Keys);

        // Table parsing happens inside the SHash contract. Loader records SHash as a dependency;
        // the table fields (including synthetic Size) belong to ISHash, not ILoader.
        Assert.True(graph.FieldUsage.TryGetValue((new ContractLabel("ISHash", "c1"), dataType),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? sHashFields));
        Assert.Contains("Table", sHashFields!.Keys);
        Assert.Contains("TableSize", sHashFields.Keys);
        Assert.Contains("Size", sHashFields.Keys);
        Assert.DoesNotContain("Table", loaderFields.Keys);
        Assert.Contains("SHash", graph.ContractsUsed[new ContractLabel("ILoader", "c1")]);
    }

    [Fact]
    public void CapturesGeneratedWritableFieldMethods()
    {
        (UsageGraph Graph, string Root)? built = BuildRealGraph();
        if (built is null) return; // cDAC source not found (running outside the repo)
        UsageGraph graph = built!.Value.Graph;

        Assert.True(graph.FieldUsage.TryGetValue(
            (new ContractLabel("ICodeNotifications", "c1"), "Data.JITNotification"),
            out IReadOnlyDictionary<string, IReadOnlyCollection<UsageKind>>? fields));
        foreach (string field in new[] { "ClrModule", "MethodToken", "State" })
        {
            Assert.True(fields!.TryGetValue(field, out IReadOnlyCollection<UsageKind>? kinds));
            Assert.Contains(UsageKind.Write, kinds!);
        }
    }

    private static HashSet<string> DataTypesUsed(UsageGraph graph, ContractLabel label) =>
        graph.FieldUsage.Keys.Where(k => k.Label == label).Select(k => k.DataType).ToHashSet();
}
