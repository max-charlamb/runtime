// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using Microsoft.Diagnostics.DataContractReader.Contracts;

namespace Microsoft.Diagnostics.DataContractReader;

/// <summary>
/// Contract registry that resolves contracts by (type, version) lookup.
/// Has no knowledge of any specific contracts — all implementations are
/// registered from outside via <see cref="Register{TContract}"/>.
/// </summary>
internal sealed class CachingContractRegistry : ContractRegistry
{
    public delegate bool TryGetContractVersionDelegate(string contractName, [NotNullWhen(true)] out string? version);

    private readonly Dictionary<Type, IContract> _contracts = [];
    private readonly Dictionary<(Type, string), Func<Target, IContract>> _creators = [];
    private readonly Target _target;
    private readonly TryGetContractVersionDelegate _tryGetContractVersion;

    public CachingContractRegistry(Target target, TryGetContractVersionDelegate tryGetContractVersion, params Action<ContractRegistry>[] contractRegistrations)
    {
        _target = target;
        _tryGetContractVersion = tryGetContractVersion;

        foreach (Action<ContractRegistry> register in contractRegistrations)
        {
            register(this);
        }
    }

    protected override void RegisterContract(Type contractType, string version, Func<Target, IContract> creator)
    {
        _creators[(contractType, version)] = creator;
    }

    protected override bool TryGetContractByType(Type contractType, string contractName, [NotNullWhen(true)] out IContract? contract, out string? failureReason)
    {
        contract = null;
        failureReason = null;
        if (_contracts.TryGetValue(contractType, out IContract? cached))
        {
            contract = cached;
            return true;
        }

        Func<Target, IContract>? creator;
        if (_tryGetContractVersion(contractName, out string? version))
        {
            if (!_creators.TryGetValue((contractType, version), out creator))
            {
                failureReason = $"Target supports contract '{contractType.Name}' version {version}, but no implementation is registered for that version.";
                return false;
            }
        }
        else if (!_creators.TryGetValue((contractType, string.Empty), out creator))
        {
            failureReason = $"Target does not support contract '{contractType.Name}'.";
            return false;
        }

        contract = creator(_target);
        if (_contracts.TryAdd(contractType, contract))
        {
            return true;
        }

        contract = _contracts[contractType];
        return true;
    }

    public override void Flush(FlushScope scope)
    {
        foreach (IContract contract in _contracts.Values)
        {
            contract.Flush(scope);
        }
    }
}
