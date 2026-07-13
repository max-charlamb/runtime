// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;

namespace CdacUsageGraph.Model;

/// <summary>
/// A single <c>CoreCLRContracts.Register&lt;IContract&gt;("cN", t =&gt; new Impl(t))</c> entry,
/// resolved to the interface name, version string, and implementation type.
/// </summary>
internal sealed record ContractRegistration(string Contract, string Version, INamedTypeSymbol Impl);
