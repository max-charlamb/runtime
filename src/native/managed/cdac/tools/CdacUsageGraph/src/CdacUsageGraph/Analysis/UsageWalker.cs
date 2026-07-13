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
public sealed class UsageWalker
{
    private readonly Microsoft.CodeAnalysis.Compilation _compilation;
    private readonly DataTypeIndex _index;
    private readonly TypeInfoCorrelator _correlator;
    private readonly SymbolEqualityComparer _cmp = SymbolEqualityComparer.Default;

    private readonly Queue<WorkItem> _queue = new();
    private readonly HashSet<(ISymbol, ContractLabel)> _visited;
    private readonly UsageCollector _collector = new();

    public UsageWalker(Microsoft.CodeAnalysis.Compilation compilation, DataTypeIndex index, TypeInfoCorrelator correlator)
    {
        _compilation = compilation;
        _index = index;
        _correlator = correlator;
        _visited = new HashSet<(ISymbol, ContractLabel)>(new MemberLabelComparer(_cmp));
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
            if (!_visited.Add((item.Member, item.Label)))
                continue;

            foreach (IOperation body in GetMemberOperations(item.Member))
                new BodyWalker(this, item.Label, item.Subst).Visit(body);
        }

        List<RegistrationInfo> regInfo = registrations
            .Select(r => new RegistrationInfo(r.Contract, r.Version, r.Impl.Name))
            .ToList();

        return new UsageGraph(cdacRoot, _index.Count, regInfo, _collector.FieldUsage, _collector.ContractsUsed);
    }

    // ---- per-operation handlers (called by the nested BodyWalker) --------------------------

    private void HandleInvocation(IInvocationOperation inv, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
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

    private void HandlePropertyReference(IPropertyReferenceOperation pr, ContractLabel label)
    {
        if (_index.IsDataType(pr.Property.ContainingType))
        {
            INamedTypeSymbol dt = pr.Property.ContainingType!;
            RecordField(label, dt, _index.NativeName(pr.Property), OperationInspector.ClassifyPropertyRef(pr));
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
                if (dt.FindImplementationForInterfaceMember(pr.Property.OriginalDefinition) is IPropertySymbol impl
                    && _index.IsField(impl))
                {
                    RecordField(label, dt, _index.NativeName(impl), kind);
                }
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
            string? dn = _correlator.GetTypeInfoDataName(pr.Instance);
            if (dn is null)
                return;
            string dataName = _index.TryGetType(dn, out INamedTypeSymbol dcls) ? DataName(dcls) : "Data." + dn;

            if (pr.Property.Name == "Fields" && OperationInspector.ExtractFieldsKey(pr) is string keyName)
            {
                // GetTypeInfo(DataType.X).Fields["nativeName"] -- a raw-string/constant field-offset lookup.
                _collector.RecordField(label, dataName, keyName, UsageKind.OffsetLookup);
            }
            else if (pr.Property.Name == "Size")
            {
                // GetTypeInfo(DataType.X).Size -- the contract depends on the descriptor's overall size.
                _collector.RecordField(label, dataName, "Size", UsageKind.Read);
            }
        }
    }

    private void HandleTypeOf(ITypeOfOperation to, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
    {
        ITypeSymbol r = Resolve(to.TypeOperand, subst);
        if (_index.IsDataType(r))
            RecordType(label, r);
    }

    // ---- recording helpers -----------------------------------------------------------------

    private static string DataName(ITypeSymbol t) => "Data." + ((INamedTypeSymbol)t.OriginalDefinition).Name;

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
        SyntaxReference? sref = member.DeclaringSyntaxReferences.FirstOrDefault();
        if (sref is null)
            yield break;
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
            case IPropertySymbol when syntax is PropertyDeclarationSyntax { Initializer.Value: { } propValue }:
                if (model.GetOperation(propValue) is IOperation propOp)
                    yield return propOp;
                break;
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
    private sealed class BodyWalker(UsageWalker owner, ContractLabel label, Dictionary<ITypeParameterSymbol, ITypeSymbol> subst)
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
            owner.HandlePropertyReference(op, label);
            base.VisitPropertyReference(op);
        }

        public override void VisitTypeOf(ITypeOfOperation op)
        {
            owner.HandleTypeOf(op, label, subst);
            base.VisitTypeOf(op);
        }
    }

    private sealed class MemberLabelComparer(SymbolEqualityComparer comparer)
        : IEqualityComparer<(ISymbol, ContractLabel)>
    {
        public bool Equals((ISymbol, ContractLabel) x, (ISymbol, ContractLabel) y) =>
            comparer.Equals(x.Item1, y.Item1) && x.Item2 == y.Item2;

        public int GetHashCode((ISymbol, ContractLabel) o) =>
            HashCode.Combine(comparer.GetHashCode(o.Item1), o.Item2);
    }
}
