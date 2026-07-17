// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Analysis.DataFlow;
using CdacUsageGraph.Discovery;
using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using Microsoft.CodeAnalysis.Operations;

namespace CdacUsageGraph.Analysis;

/// <summary>
/// Phase C/D: the forward interprocedural walk. Seeds from each contract implementation's members
/// (methods, constructors, and field/property initializers), propagates a <see cref="ContractLabel"/>
/// through callees, constructed helpers, base/generic-base classes and static-abstract dispatch,
/// and collects per-field usage into an immutable <see cref="UsageGraph"/>.
/// </summary>
internal sealed class UsageWalker
{
    private readonly CSharpCompilation _compilation;
    private readonly DataTypeIndex _index;
    private readonly DataFlowTypeInfoResolver? _dataFlowResolver;
    private readonly SymbolEqualityComparer _cmp = SymbolEqualityComparer.Default;

    private readonly Queue<WorkItem> _queue = new();
    private readonly HashSet<WorkItem> _visited;
    private readonly Dictionary<ContractLabel, HashSet<INamedTypeSymbol>> _constructedTypes = [];
    private readonly Dictionary<ContractLabel, List<PendingDispatch>> _pendingDispatches = [];
    private readonly UsageCollector _collector = new();

    public UsageWalker(
        CSharpCompilation compilation,
        DataTypeIndex index,
        DataFlowTypeInfoResolver dataFlowResolver)
    {
        _compilation = compilation;
        _index = index;
        _dataFlowResolver = dataFlowResolver;
        _visited = new HashSet<WorkItem>(new WorkItemComparer(_cmp));
    }

    internal UsageWalker(
        CSharpCompilation compilation,
        DataTypeIndex index)
    {
        _compilation = compilation;
        _index = index;
        _visited = new HashSet<WorkItem>(new WorkItemComparer(_cmp));
    }

    internal UsageGraph Walk(IReadOnlyList<ContractRegistration> registrations, string cdacRoot)
    {
        foreach (ContractRegistration reg in registrations)
        {
            ContractLabel label = new ContractLabel(reg.Contract, reg.Version);
            AddConstructedType(label, reg.Impl);
            Dictionary<ITypeParameterSymbol, ITypeSymbol> seed =
                GenericDispatch.BuildSubstitutions(
                    reg.Impl,
                    new Dictionary<ITypeParameterSymbol, ITypeSymbol>(_cmp),
                    _cmp);
            foreach (ISymbol m in ContractEntryPointDiscovery.Discover(reg))
                _queue.Enqueue(new WorkItem(m, label, seed, RecordDataFlowEffects: true));
        }

        while (_queue.Count > 0)
        {
            WorkItem item = _queue.Dequeue();
            if (!_visited.Add(item))
                continue;

            if (item.Member is IMethodSymbol reachableMethod)
            {
                _collector.RecordReachableMethod(
                    item.Label,
                    FormatMethod(reachableMethod, item.Subst));
            }
            if (item.RecordDataFlowEffects)
                RecordDataFlowEffects(item.Member, item.Label);
            foreach (IOperation body in GetMemberOperations(item.Member))
            {
                new BodyWalker(
                    this,
                    item.Member,
                    item.Label,
                    item.Subst).Visit(body);
            }
        }

        List<RegistrationInfo> regInfo = registrations
            .Select(r => new RegistrationInfo(r.Contract, r.Version, r.Impl.Name))
            .ToList();

        return new UsageGraph(
            cdacRoot,
            _index.Count,
            regInfo,
            _collector.FieldUsage,
            _collector.FieldTypes,
            _collector.GlobalUsage,
            _collector.ContractsUsed,
            _collector.ReachableMethods);
    }

    // ---- per-operation handlers (called by the nested BodyWalker) --------------------------

    private void HandleInvocation(IInvocationOperation inv, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        foreach (ITypeSymbol ta in inv.TargetMethod.TypeArguments)
        {
            ITypeSymbol r = GenericDispatch.Resolve(ta, subst);
            if (_index.IsDataType(r))
            {
                RecordType(label, r);
                RecordDataFactoryGlobals(r, label);
            }
        }

        IMethodSymbol callee = inv.TargetMethod;
        if (IsCrossContractInvocation(callee, label))
            EnqueueEscapingCallbacks(inv, label, subst);
        if (RequiresDynamicDispatch(callee) &&
            ShouldFollowDynamicDispatch(callee, label))
        {
            AddPendingDispatch(callee, label, subst);
        }
        else if (_cmp.Equals(
            callee.OriginalDefinition.ContainingAssembly,
            _compilation.Assembly) &&
            !callee.IsAbstract &&
            callee.ContainingType.TypeKind != TypeKind.Interface)
        {
            EnqueueMember(callee, label, subst);
        }

        // Static-abstract / type-parameter dispatch (e.g. TImpl.StubPrecode_GetMethodDesc(...)):
        // resolve the receiver type parameter to its concrete substitution and enqueue the impl.
        IMethodSymbol? concrete = GenericDispatch.ResolveStaticAbstractTarget(
            _compilation,
            inv,
            subst);
        if (concrete is not null)
        {
            EnqueueMember(concrete, label, subst, recordDataFlowEffects: true);
        }
    }

    private void RecordDataFlowEffects(ISymbol member, ContractLabel label)
    {
        if (_dataFlowResolver is null)
            return;

        foreach (FieldAccessEffect effect in _dataFlowResolver.GetFieldAccessEffects(member))
        {
            string dataName = _index.TryGetType(effect.Field.TypeName, out DataTypeInfo dataType)
                ? DataName(dataType)
                : "Data." + effect.Field.TypeName;
            string fieldType = dataType?.GetNativeFieldType(effect.Field.FieldName)
                ?? (effect.Field.FieldName == "Size" ? "uint32" : "unknown");
            _collector.RecordField(
                label,
                dataName,
                effect.Field.FieldName,
                fieldType,
                effect.Usage);
        }
        foreach (GlobalAccessEffect effect in
            _dataFlowResolver.GetGlobalAccessEffects(member))
        {
            _collector.RecordGlobal(
                label,
                effect.Name,
                effect.Type,
                effect.IsOptional);
        }
    }

    private void RecordDataFactoryGlobals(
        ITypeSymbol type,
        ContractLabel label)
    {
        if (_dataFlowResolver is null ||
            type is not INamedTypeSymbol dataType ||
            GenericDispatch.FindDataFactory(dataType) is not IMethodSymbol factory)
        {
            return;
        }

        foreach (GlobalAccessEffect effect in
            _dataFlowResolver.GetGlobalAccessEffects(factory))
        {
            _collector.RecordGlobal(
                label,
                effect.Name,
                effect.Type,
                effect.IsOptional);
        }
    }

    private void HandleObjectCreation(IObjectCreationOperation oc, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        if (oc.Type is not INamedTypeSymbol ct)
            return;
        AddConstructedType(label, ct);
        if (_index.IsDataType(ct))
        {
            RecordType(label, ct);
        }
        else if (_cmp.Equals(ct.OriginalDefinition.ContainingAssembly, _compilation.Assembly))
        {
            // Construction reaches the selected constructor and the type's initializers. Other
            // helper methods become reachable through invocation edges.
            Dictionary<ITypeParameterSymbol, ITypeSymbol> sub =
                GenericDispatch.BuildSubstitutions(ct, subst, _cmp);
            foreach (ISymbol m in ContractEntryPointDiscovery.DiscoverConstruction(
                ct,
                oc.Constructor))
            {
                _queue.Enqueue(new WorkItem(m, label, sub, RecordDataFlowEffects: true));
            }
        }
    }

    private void HandlePropertyReference(
        IPropertyReferenceOperation pr,
        ISymbol containingMember,
        ContractLabel label)
    {
        if (_index.TryGetDataType(pr.Property.ContainingType, out DataTypeInfo directDataType))
        {
            UsageKind kind = OperationInspector.ClassifyPropertyRef(pr);
            if (kind == UsageKind.Write &&
                containingMember is IMethodSymbol { MethodKind: MethodKind.Constructor } constructor &&
                _cmp.Equals(constructor.ContainingType, pr.Property.ContainingType))
            {
                // Generated Data constructors assign each property from target memory. The
                // assignment target is a C# write, but its descriptor dependency is a target read.
                kind = UsageKind.Read;
            }
            HandleDataProperty(
                pr.Property,
                directDataType,
                kind,
                label);
        }
        else if (pr.Property.ContainingType is { TypeKind: TypeKind.Interface } iface)
        {
            // A read through an interface implemented by Data types (e.g. an IExceptionClauseData
            // local that may hold either R2RExceptionClause or EEExceptionClause at runtime). We
            // can't know the concrete type statically, so -- conservatively -- attribute the read to
            // every implementing Data type whose implementation of this member is an actual [Field]
            // (computed/pass-through properties map to no descriptor field and are skipped).
            UsageKind kind = OperationInspector.ClassifyPropertyRef(pr);
            foreach (DataTypeInfo dataType in _index.DataTypesImplementing(iface))
            {
                if (dataType.Symbol.FindImplementationForInterfaceMember(pr.Property.OriginalDefinition) is IPropertySymbol impl)
                    HandleDataProperty(impl, dataType, kind, label);
            }
        }
        else if (pr.Property.ContainingType?.Name == "ContractRegistry")
        {
            // _target.Contracts.<X> -- a contract dependency (goes in "Contracts used", not data
            // descriptors). The callee's own reads are not attributed here (interface throw-stub body).
            _collector.RecordContractUsed(label, pr.Property.Name);
        }
    }

    private void HandleDataProperty(
        IPropertySymbol property,
        DataTypeInfo dataType,
        UsageKind kind,
        ContractLabel label)
    {
        DataPropertyInfo info = dataType.GetProperty(property);
        if (info.Kind == DataPropertyKind.DirectField)
        {
            RecordField(
                label,
                dataType,
                info.NativeName,
                info.NativeType,
                kind);
            return;
        }
        if (info.Kind == DataPropertyKind.TypeSize)
        {
            RecordField(
                label,
                dataType,
                "Size",
                "uint32",
                UsageKind.Read);
            return;
        }

        RecordType(label, dataType);
        foreach (ISymbol member in info.ExpansionMembers)
            _queue.Enqueue(new WorkItem(member, label,
                new Dictionary<ITypeParameterSymbol, ITypeSymbol>(_cmp),
                RecordDataFlowEffects: true));
    }

    private void HandleTypeOf(ITypeOfOperation to, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        ITypeSymbol r = GenericDispatch.Resolve(to.TypeOperand, subst);
        if (_index.IsDataType(r))
            RecordType(label, r);
    }

    private void HandleMethodReference(
        IMethodReferenceOperation methodReference,
        ContractLabel label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> substitutions)
    {
        IMethodSymbol method = methodReference.Method;
        if (_cmp.Equals(
            method.OriginalDefinition.ContainingAssembly,
            _compilation.Assembly))
        {
            EnqueueMember(
                method,
                label,
                substitutions,
                recordDataFlowEffects: true);
        }
    }

    // ---- recording helpers -----------------------------------------------------------------

    private static string DataName(DataTypeInfo type) => "Data." + type.Name;

    private static string FormatMethod(
        IMethodSymbol method,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> substitutions)
    {
        string display = method.ToDisplayString(
            SymbolDisplayFormat.CSharpErrorMessageFormat);
        if (substitutions.Count == 0)
            return display;

        string binding = string.Join(
            ", ",
            substitutions
                .OrderBy(entry => entry.Key.ToDisplayString(), StringComparer.Ordinal)
                .Select(entry =>
                    $"{entry.Key.Name}={entry.Value.ToDisplayString(SymbolDisplayFormat.CSharpErrorMessageFormat)}"));
        return $"{display} [{binding}]";
    }

    private void RecordType(ContractLabel label, ITypeSymbol t) =>
        _collector.RecordType(label, _index.TryGetDataType(t, out DataTypeInfo info) ? DataName(info) : "Data." + t.Name);

    private void RecordType(ContractLabel label, DataTypeInfo type) =>
        _collector.RecordType(label, DataName(type));

    private void RecordField(
        ContractLabel label,
        DataTypeInfo dataType,
        string field,
        string type,
        UsageKind kind) =>
        _collector.RecordField(label, DataName(dataType), field, type, kind);

    // ---- member enumeration & body resolution ----------------------------------------------

    // The IOperation(s) to walk for a member: a method body, or a field/property initializer value.
    private IEnumerable<IOperation> GetMemberOperations(ISymbol member)
    {
        foreach (SyntaxReference sref in member.DeclaringSyntaxReferences)
        {
            SyntaxNode syntax = sref.GetSyntax();
            SemanticModel model = _compilation.GetSemanticModel(syntax.SyntaxTree);

            switch (member)
            {
                case IMethodSymbol:
                    if (model.GetOperation(syntax) is IOperation methodOp)
                        yield return methodOp;
                    break;
                case IFieldSymbol when syntax is VariableDeclaratorSyntax { Initializer.Value: { } fieldValue }:
                    if (model.GetOperation(fieldValue) is IOperation fieldOp)
                        yield return fieldOp;
                    break;
                case IPropertySymbol when syntax is PropertyDeclarationSyntax pds:
                    // Computed getter of a Data property: walk the initializer (= expr), expression body
                    // (=> expr) and accessor bodies so the actual [Field]s it reads are attributed.
                    if (pds.Initializer?.Value is { } initValue && model.GetOperation(initValue) is { } initOp)
                        yield return initOp;
                    if (pds.ExpressionBody?.Expression is { } exprValue && model.GetOperation(exprValue) is { } exprOp)
                        yield return exprOp;
                    if (pds.AccessorList is not { } accessors)
                        break;
                    foreach (AccessorDeclarationSyntax accessor in accessors.Accessors.Where(a =>
                        a.Kind() == Microsoft.CodeAnalysis.CSharp.SyntaxKind.GetAccessorDeclaration))
                    {
                        SyntaxNode? body = (SyntaxNode?)accessor.Body ?? accessor.ExpressionBody?.Expression;
                        if (body is not null && model.GetOperation(body) is { } accessorOp)
                            yield return accessorOp;
                    }
                    break;
            }
        }
    }

    // ---- interprocedural machinery ---------------------------------------------------------

    private void EnqueueMember(
        IMethodSymbol callee,
        ContractLabel label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> outer,
        bool recordDataFlowEffects = false)
    {
        IMethodSymbol def = callee.OriginalDefinition;
        Dictionary<ITypeParameterSymbol, ITypeSymbol> sub = new Dictionary<ITypeParameterSymbol, ITypeSymbol>(_cmp);
        if (callee.ContainingType is INamedTypeSymbol ct)
            foreach (KeyValuePair<ITypeParameterSymbol, ITypeSymbol> kv in
                GenericDispatch.BuildSubstitutions(ct, outer, _cmp))
                sub[kv.Key] = kv.Value;
        for (int i = 0; i < def.TypeParameters.Length && i < callee.TypeArguments.Length; i++)
            sub[def.TypeParameters[i]] = GenericDispatch.Resolve(
                callee.TypeArguments[i],
                outer);
        _queue.Enqueue(new WorkItem(def, label, sub, recordDataFlowEffects));
    }

    private static bool RequiresDynamicDispatch(IMethodSymbol method) =>
        !method.IsStatic &&
        (method.IsAbstract ||
            method.IsVirtual ||
            method.ContainingType.TypeKind == TypeKind.Interface);

    private static bool ShouldFollowDynamicDispatch(
        IMethodSymbol method,
        ContractLabel label)
    {
        INamedTypeSymbol? containingType = method.ContainingType;
        if (containingType?.TypeKind != TypeKind.Interface)
            return true;

        bool isContract = containingType.AllInterfaces.Any(@interface =>
            @interface.Name == "IContract");
        return !isContract || containingType.Name == label.Contract;
    }

    private static bool IsCrossContractInvocation(
        IMethodSymbol method,
        ContractLabel label)
    {
        INamedTypeSymbol? containingType = method.ContainingType;
        return containingType?.TypeKind == TypeKind.Interface &&
            containingType.Name != label.Contract &&
            containingType.AllInterfaces.Any(@interface =>
                @interface.Name == "IContract");
    }

    private void EnqueueEscapingCallbacks(
        IInvocationOperation invocation,
        ContractLabel label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> substitutions)
    {
        foreach (IArgumentOperation argument in invocation.Arguments)
        {
            if (argument.Parameter?.Type is not INamedTypeSymbol
                {
                    TypeKind: TypeKind.Interface,
                } callbackInterface ||
                callbackInterface.AllInterfaces.Any(@interface =>
                    @interface.Name == "IContract"))
            {
                continue;
            }

            ITypeSymbol? operandType = OperationInspector.Unwrap(argument.Value).Type;
            if (operandType is INamedTypeSymbol concreteType &&
                _cmp.Equals(
                    concreteType.OriginalDefinition.ContainingAssembly,
                    _compilation.Assembly))
            {
                EnqueueInterfaceMembers(
                    concreteType,
                    callbackInterface,
                    label,
                    substitutions);
            }

            if (_constructedTypes.TryGetValue(
                label,
                out HashSet<INamedTypeSymbol>? constructedTypes))
            {
                foreach (INamedTypeSymbol constructedType in constructedTypes)
                {
                    EnqueueInterfaceMembers(
                        constructedType,
                        callbackInterface,
                        label,
                        substitutions);
                }
            }
        }
    }

    private void EnqueueInterfaceMembers(
        INamedTypeSymbol concreteType,
        INamedTypeSymbol callbackInterface,
        ContractLabel label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> substitutions)
    {
        if (!CanDispatchTo(concreteType, callbackInterface))
            return;

        foreach (IMethodSymbol interfaceMethod in callbackInterface.GetMembers()
            .OfType<IMethodSymbol>())
        {
            IMethodSymbol? implementation =
                GenericDispatch.FindInterfaceImplementation(
                    concreteType,
                    interfaceMethod);
            if (implementation is not null)
            {
                EnqueueMember(
                    implementation,
                    label,
                    substitutions,
                    recordDataFlowEffects: true);
            }
        }
    }

    private void AddConstructedType(ContractLabel label, INamedTypeSymbol type)
    {
        if (!_constructedTypes.TryGetValue(label, out HashSet<INamedTypeSymbol>? types))
        {
            _constructedTypes[label] = types =
                new HashSet<INamedTypeSymbol>(_cmp);
        }
        if (!types.Add(type))
            return;

        if (_pendingDispatches.TryGetValue(label, out List<PendingDispatch>? pending))
        {
            foreach (PendingDispatch dispatch in pending)
                EnqueueDispatchTarget(type, dispatch);
        }
    }

    private void AddPendingDispatch(
        IMethodSymbol method,
        ContractLabel label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> substitutions)
    {
        PendingDispatch dispatch = new(
            method,
            label,
            new Dictionary<ITypeParameterSymbol, ITypeSymbol>(substitutions, _cmp));
        if (!_pendingDispatches.TryGetValue(label, out List<PendingDispatch>? pending))
            _pendingDispatches[label] = pending = [];
        pending.Add(dispatch);

        if (_constructedTypes.TryGetValue(label, out HashSet<INamedTypeSymbol>? types))
        {
            foreach (INamedTypeSymbol type in types)
                EnqueueDispatchTarget(type, dispatch);
        }
    }

    private void EnqueueDispatchTarget(
        INamedTypeSymbol constructedType,
        PendingDispatch dispatch)
    {
        IMethodSymbol method = dispatch.Method;
        INamedTypeSymbol? declaringType = method.ContainingType;
        if (declaringType is null ||
            !CanDispatchTo(constructedType, declaringType))
        {
            return;
        }

        IMethodSymbol? implementation =
            declaringType.TypeKind == TypeKind.Interface
                ? GenericDispatch.FindInterfaceImplementation(
                    constructedType,
                    method)
                : GenericDispatch.FindVirtualImplementation(
                    constructedType,
                    method);
        if (implementation is not null &&
            _cmp.Equals(
                implementation.OriginalDefinition.ContainingAssembly,
                _compilation.Assembly))
        {
            EnqueueMember(
                implementation,
                dispatch.Label,
                dispatch.Substitutions,
                recordDataFlowEffects: true);
        }
    }

    private bool CanDispatchTo(
        INamedTypeSymbol constructedType,
        INamedTypeSymbol declaringType)
    {
        if (_cmp.Equals(
            constructedType.OriginalDefinition,
            declaringType.OriginalDefinition))
        {
            return true;
        }
        if (constructedType.AllInterfaces.Any(@interface =>
            _cmp.Equals(
                @interface.OriginalDefinition,
                declaringType.OriginalDefinition)))
        {
            return true;
        }
        for (INamedTypeSymbol? current = constructedType.BaseType;
            current is not null;
            current = current.BaseType)
        {
            if (_cmp.Equals(
                current.OriginalDefinition,
                declaringType.OriginalDefinition))
            {
                return true;
            }
        }
        return false;
    }

    // ---- nested types ----------------------------------------------------------------------

    private sealed record WorkItem(
        ISymbol Member,
        ContractLabel Label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> Subst,
        bool RecordDataFlowEffects);

    private sealed record PendingDispatch(
        IMethodSymbol Method,
        ContractLabel Label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> Substitutions);

    /// <summary>Depth-first per-body walker; dispatches back to the owning <see cref="UsageWalker"/>.</summary>
    private sealed class BodyWalker(
        UsageWalker owner,
        ISymbol containingMember,
        ContractLabel label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
        : OperationWalker
    {
        public override void VisitInvocation(IInvocationOperation op)
        {
            owner.HandleInvocation(op, label, subst);
            base.VisitInvocation(op);
        }

        public override void VisitObjectCreation(IObjectCreationOperation op)
        {
            owner.HandleObjectCreation(op, label, subst);
            base.VisitObjectCreation(op);
        }

        public override void VisitPropertyReference(IPropertyReferenceOperation op)
        {
            owner.HandlePropertyReference(op, containingMember, label);
            base.VisitPropertyReference(op);
        }

        public override void VisitTypeOf(ITypeOfOperation op)
        {
            owner.HandleTypeOf(op, label, subst);
            base.VisitTypeOf(op);
        }

        public override void VisitMethodReference(IMethodReferenceOperation op)
        {
            owner.HandleMethodReference(op, label, subst);
            base.VisitMethodReference(op);
        }

    }

    private sealed class WorkItemComparer(SymbolEqualityComparer comparer)
        : IEqualityComparer<WorkItem>
    {
        public bool Equals(WorkItem? x, WorkItem? y)
        {
            if (ReferenceEquals(x, y))
                return true;
            if (x is null || y is null ||
                !comparer.Equals(x.Member, y.Member) ||
                x.Label != y.Label ||
                x.RecordDataFlowEffects != y.RecordDataFlowEffects ||
                x.Subst.Count != y.Subst.Count)
                return false;

            foreach (KeyValuePair<ITypeParameterSymbol, ITypeSymbol> entry in x.Subst)
            {
                if (!y.Subst.TryGetValue(entry.Key, out ITypeSymbol? other) ||
                    !comparer.Equals(entry.Value, other))
                    return false;
            }
            return true;
        }

        public int GetHashCode(WorkItem item)
        {
            int substitutionsHash = 0;
            foreach (KeyValuePair<ITypeParameterSymbol, ITypeSymbol> entry in item.Subst)
            {
                // XOR makes the aggregate independent of dictionary iteration order.
                substitutionsHash ^= HashCode.Combine(
                    comparer.GetHashCode(entry.Key),
                    comparer.GetHashCode(entry.Value));
            }
            return HashCode.Combine(
                comparer.GetHashCode(item.Member),
                item.Label,
                substitutionsHash,
                item.RecordDataFlowEffects);
        }
    }
}
