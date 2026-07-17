// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.FlowAnalysis;

namespace CdacUsageGraph.Analysis.DataFlow;

internal readonly struct FlowSlot
{
    private FlowSlot(ISymbol? symbol, CaptureId? capture)
    {
        Symbol = symbol;
        Capture = capture;
    }

    public ISymbol? Symbol { get; }

    public CaptureId? Capture { get; }

    public static FlowSlot ForSymbol(ISymbol symbol) => new(symbol.OriginalDefinition, null);

    public static FlowSlot ForCapture(CaptureId capture) => new(null, capture);
}

internal sealed class FlowSlotComparer : IEqualityComparer<FlowSlot>
{
    public static FlowSlotComparer Instance { get; } = new();

    public bool Equals(FlowSlot x, FlowSlot y)
    {
        if (x.Capture.HasValue || y.Capture.HasValue)
            return Nullable.Equals(x.Capture, y.Capture);
        return SymbolEqualityComparer.Default.Equals(x.Symbol, y.Symbol);
    }

    public int GetHashCode(FlowSlot slot) =>
        slot.Capture?.GetHashCode() ??
        (slot.Symbol is null ? 0 : SymbolEqualityComparer.Default.GetHashCode(slot.Symbol));
}
