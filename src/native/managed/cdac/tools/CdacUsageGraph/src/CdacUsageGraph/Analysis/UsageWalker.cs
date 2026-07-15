// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using CdacUsageGraph.Discovery;
using CdacUsageGraph.Model;
using Microsoft.CodeAnalysis;
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
    private readonly Microsoft.CodeAnalysis.Compilation _compilation;
    private readonly DataTypeIndex _index;
    private readonly TypeInfoCorrelator _correlator;
    private readonly SymbolEqualityComparer _cmp = SymbolEqualityComparer.Default;

    private readonly Queue<WorkItem> _queue = new();
    private readonly HashSet<WorkItem> _visited;
    private readonly UsageCollector _collector = new();

    public UsageWalker(Microsoft.CodeAnalysis.Compilation compilation, DataTypeIndex index, TypeInfoCorrelator correlator)
    {
        _compilation = compilation;
        _index = index;
        _correlator = correlator;
        _visited = new HashSet<WorkItem>(new WorkItemComparer(_cmp));
    }

    internal UsageGraph Walk(IReadOnlyList<ContractRegistration> registrations, string cdacRoot)
    {
        foreach (ContractRegistration reg in registrations)
        {
            ContractLabel label = new ContractLabel(reg.Contract, reg.Version);
            Dictionary<ITypeParameterSymbol, ITypeSymbol> seed = BuildSubst(reg.Impl, new Dictionary<ITypeParameterSymbol, ITypeSymbol>(_cmp));
            foreach (ISymbol m in AllMembers(reg.Impl))
                _queue.Enqueue(new WorkItem(m, label, seed));
        }

        while (_queue.Count > 0)
        {
            WorkItem item = _queue.Dequeue();
            if (!_visited.Add(item))
                continue;

            foreach (IOperation body in GetMemberOperations(item.Member))
                new BodyWalker(this, item.Member, item.Label, item.Subst).Visit(body);
        }

        List<RegistrationInfo> regInfo = registrations
            .Select(r => new RegistrationInfo(r.Contract, r.Version, r.Impl.Name))
            .ToList();

        return new UsageGraph(cdacRoot, _index.Count, regInfo, _collector.FieldUsage, _collector.ContractsUsed);
    }

    // ---- per-operation handlers (called by the nested BodyWalker) --------------------------

    private void HandleInvocation(IInvocationOperation inv, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        HandleTypeInfoFieldRead(inv, label);

        foreach (ITypeSymbol ta in inv.TargetMethod.TypeArguments)
        {
            ITypeSymbol r = Resolve(ta, subst);
            if (_index.IsDataType(r))
                RecordType(label, r);
        }

        IMethodSymbol callee = inv.TargetMethod;
        if (_cmp.Equals(callee.OriginalDefinition.ContainingAssembly, _compilation.Assembly))
            EnqueueMember(callee, label, subst);

        // Static-abstract / type-parameter dispatch (e.g. TImpl.StubPrecode_GetMethodDesc(...)):
        // resolve the receiver type parameter to its concrete substitution and enqueue the impl.
        ITypeSymbol? receiver = inv.ConstrainedToType
            ?? (callee.ContainingType is ITypeParameterSymbol ? callee.ContainingType : null);
        if (receiver is not null && Resolve(receiver, subst) is INamedTypeSymbol implType &&
            _cmp.Equals(implType.OriginalDefinition.ContainingAssembly, _compilation.Assembly))
        {
            IMethodSymbol? concrete = FindConcreteMethod(implType, callee);
            if (concrete is not null)
                EnqueueMember(concrete, label, subst);
        }
    }

    // Target.Read*Field(address, typeInfo, nameof(Field)): a descriptor-field read expressed
    // through the Target helper API rather than TypeInfo.Fields["..."]. The correlator follows the
    // TypeInfo identity through aliases and helper parameters/fields, so reusable helpers such as
    // DacEnumerableHash attribute these fields to every concrete Data type that can flow in.
    private void HandleTypeInfoFieldRead(IInvocationOperation invocation, ContractLabel label)
    {
        if (!invocation.TargetMethod.Name.StartsWith("Read", StringComparison.Ordinal) ||
            !invocation.TargetMethod.Name.Contains("Field", StringComparison.Ordinal))
            return;

        IArgumentOperation? typeInfoArgument = invocation.Arguments.FirstOrDefault(
            a => a.Parameter?.Type.Name == "TypeInfo" && a.Parameter.Type.ContainingType?.Name == "Target");
        IArgumentOperation? fieldNameArgument = invocation.Arguments.FirstOrDefault(
            a => a.Value.ConstantValue is { HasValue: true, Value: string });
        if (typeInfoArgument is null ||
            fieldNameArgument?.Value.ConstantValue is not { HasValue: true, Value: string fieldName })
            return;

        foreach (string dataTypeName in _correlator.GetTypeInfoDataNames(typeInfoArgument.Value))
        {
            string dataName = _index.TryGetType(dataTypeName, out INamedTypeSymbol dataType)
                ? DataName(dataType)
                : "Data." + dataTypeName;
            _collector.RecordField(label, dataName, fieldName, UsageKind.Read);
        }
    }

    private void HandleObjectCreation(IObjectCreationOperation oc, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        if (oc.Type is not INamedTypeSymbol ct)
            return;
        if (_index.IsDataType(ct))
        {
            RecordType(label, ct);
        }
        else if (_cmp.Equals(ct.OriginalDefinition.ContainingAssembly, _compilation.Assembly))
        {
            // A contract that constructs an in-assembly helper "uses" it: walk it.
            Dictionary<ITypeParameterSymbol, ITypeSymbol> sub = BuildSubst(ct, subst);
            foreach (ISymbol m in AllMembers(ct))
                _queue.Enqueue(new WorkItem(m, label, sub));
        }
    }

    private void HandlePropertyReference(
        IPropertyReferenceOperation pr,
        ISymbol containingMember,
        ContractLabel label)
    {
        if (_index.IsDataType(pr.Property.ContainingType))
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
                pr.Property.ContainingType!,
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
            foreach (INamedTypeSymbol dt in _index.DataTypesImplementing(iface))
            {
                if (dt.FindImplementationForInterfaceMember(pr.Property.OriginalDefinition) is IPropertySymbol impl)
                    HandleDataProperty(impl, dt, kind, label);
            }
        }
        else if (pr.Property.ContainingType?.Name == "ContractRegistry")
        {
            // _target.Contracts.<X> -- a contract dependency (goes in "Contracts used", not data
            // descriptors). The callee's own reads are not attributed here (interface throw-stub body).
            _collector.RecordContractUsed(label, pr.Property.Name);
        }
        else if (pr.Property.ContainingType?.Name == "TypeInfo")
        {
            // A layout reference on Target.TypeInfo. Resolve which Data type it describes.
            IReadOnlyCollection<string> dataTypeNames = _correlator.GetTypeInfoDataNames(pr.Instance);
            if (dataTypeNames.Count == 0)
                return;

            if (pr.Property.Name == "Fields" && OperationInspector.ExtractFieldsKey(pr) is string keyName)
            {
                // GetTypeInfo(DataType.X).Fields["nativeName"] -- a raw-string/constant field-offset lookup.
                foreach (string dataTypeName in dataTypeNames)
                {
                    string dataName = _index.TryGetType(dataTypeName, out INamedTypeSymbol dataType)
                        ? DataName(dataType)
                        : "Data." + dataTypeName;
                    _collector.RecordField(label, dataName, keyName, UsageKind.OffsetLookup);
                }
            }

            else if (pr.Property.Name == "Size")
            {
                // GetTypeInfo(DataType.X).Size -- the contract depends on the descriptor's overall size.
                foreach (string dataTypeName in dataTypeNames)
                {
                    string dataName = _index.TryGetType(dataTypeName, out INamedTypeSymbol dataType)
                        ? DataName(dataType)
                        : "Data." + dataTypeName;
                    _collector.RecordField(label, dataName, "Size", UsageKind.Read);
                }
            }
        }
    }

    private void HandleDataProperty(
        IPropertySymbol property,
        INamedTypeSymbol dataType,
        UsageKind kind,
        ContractLabel label)
    {
        DataPropertyInfo info = _index.GetPropertyInfo(property);
        if (info.Kind == DataPropertyKind.DirectField)
        {
            RecordField(label, dataType, info.NativeName, kind);
            return;
        }
        if (info.Kind == DataPropertyKind.TypeSize)
        {
            RecordField(label, dataType, "Size", UsageKind.Read);
            return;
        }

        RecordType(label, dataType);
        foreach (ISymbol member in info.ExpansionMembers)
            _queue.Enqueue(new WorkItem(member, label,
                new Dictionary<ITypeParameterSymbol, ITypeSymbol>(_cmp)));
    }

    private void HandleTypeOf(ITypeOfOperation to, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        ITypeSymbol r = Resolve(to.TypeOperand, subst);
        if (_index.IsDataType(r))
            RecordType(label, r);
    }

    // ---- recording helpers -----------------------------------------------------------------

    private string DataName(ITypeSymbol t) => "Data." + _index.DescriptorName(t);

    private void RecordType(ContractLabel label, ITypeSymbol t) => _collector.RecordType(label, DataName(t));

    private void RecordField(ContractLabel label, ITypeSymbol dataType, string field, UsageKind kind) =>
        _collector.RecordField(label, DataName(dataType), field, kind);

    // ---- member enumeration & body resolution ----------------------------------------------

    // All walkable members declared on a type and its base chain (as original definitions): methods
    // and constructors, plus fields/properties (which may carry initializers to walk).
    private IEnumerable<ISymbol> AllMembers(INamedTypeSymbol t)
    {
        INamedTypeSymbol? cur = t;
        while (cur is not null && _cmp.Equals(cur.ContainingAssembly, _compilation.Assembly))
        {
            foreach (ISymbol m in cur.OriginalDefinition.GetMembers())
            {
                switch (m)
                {
                    case IMethodSymbol method
                        when method.MethodKind is MethodKind.Ordinary or MethodKind.Constructor
                            or MethodKind.ExplicitInterfaceImplementation or MethodKind.PropertyGet
                            or MethodKind.PropertySet:
                        yield return method;
                        break;
                    case IFieldSymbol { IsConst: false, IsImplicitlyDeclared: false } field:
                        yield return field;
                        break;
                    case IPropertySymbol prop:
                        yield return prop;
                        break;
                }
            }
            cur = cur.BaseType;
        }
    }

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
                    foreach (AccessorDeclarationSyntax accessor in accessors.Accessors)
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

    // Transitive type-parameter resolution through a substitution map.
    private static ITypeSymbol Resolve(ITypeSymbol t, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        int guard = 0;
        while (t is ITypeParameterSymbol tp && subst.TryGetValue(tp, out ITypeSymbol? mapped) && guard++ < 16)
            t = mapped;
        return t;
    }

    // Substitution for a constructed named type, composing type arguments through an outer
    // substitution first. Traverses both the base-type chain AND the containing-type chain (so
    // type parameters of an enclosing generic type, e.g. a nested helper class, are captured).
    private Dictionary<ITypeParameterSymbol, ITypeSymbol> BuildSubst(
        INamedTypeSymbol constructed, Dictionary<ITypeParameterSymbol, ITypeSymbol> outer)
    {
        Dictionary<ITypeParameterSymbol, ITypeSymbol> d = new Dictionary<ITypeParameterSymbol, ITypeSymbol>(_cmp);
        Stack<INamedTypeSymbol> stack = new Stack<INamedTypeSymbol>();
        HashSet<INamedTypeSymbol> seen = new HashSet<INamedTypeSymbol>(_cmp);
        stack.Push(constructed);
        while (stack.Count > 0)
        {
            INamedTypeSymbol cur = stack.Pop();
            if (!seen.Add(cur))
                continue;
            INamedTypeSymbol def = cur.OriginalDefinition;
            for (int i = 0; i < def.TypeParameters.Length && i < cur.TypeArguments.Length; i++)
                d[def.TypeParameters[i]] = Resolve(cur.TypeArguments[i], outer);
            if (cur.BaseType is INamedTypeSymbol bt)
                stack.Push(bt);
            if (cur.ContainingType is INamedTypeSymbol ct)
                stack.Push(ct);
        }
        return d;
    }

    private void EnqueueMember(IMethodSymbol callee, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> outer)
    {
        IMethodSymbol def = callee.OriginalDefinition;
        Dictionary<ITypeParameterSymbol, ITypeSymbol> sub = new Dictionary<ITypeParameterSymbol, ITypeSymbol>(_cmp);
        if (callee.ContainingType is INamedTypeSymbol ct)
            foreach (KeyValuePair<ITypeParameterSymbol, ITypeSymbol> kv in BuildSubst(ct, outer))
                sub[kv.Key] = kv.Value;
        for (int i = 0; i < def.TypeParameters.Length && i < callee.TypeArguments.Length; i++)
            sub[def.TypeParameters[i]] = Resolve(callee.TypeArguments[i], outer);
        _queue.Enqueue(new WorkItem(def, label, sub));
    }

    // Concrete method (by name + arity) implementing/overriding `callee` on `implType` or its base
    // chain. Resolves static-abstract dispatch through a generic type parameter.
    private IMethodSymbol? FindConcreteMethod(INamedTypeSymbol implType, IMethodSymbol callee)
    {
        INamedTypeSymbol? cur = implType;
        while (cur is not null && _cmp.Equals(cur.OriginalDefinition.ContainingAssembly, _compilation.Assembly))
        {
            foreach (IMethodSymbol m in cur.OriginalDefinition.GetMembers(callee.Name).OfType<IMethodSymbol>())
                if (m.Parameters.Length == callee.Parameters.Length)
                    return m;
            cur = cur.BaseType;
        }
        return implType.FindImplementationForInterfaceMember(callee.OriginalDefinition) as IMethodSymbol;
    }

    // ---- nested types ----------------------------------------------------------------------

    private sealed record WorkItem(
        ISymbol Member,
        ContractLabel Label,
        Dictionary<ITypeParameterSymbol, ITypeSymbol> Subst);

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
                comparer.GetHashCode(item.Member), item.Label, substitutionsHash);
        }
    }
}
