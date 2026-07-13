// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.CommandLine;

namespace CdacUsageGraph;

/// <summary>Defines the command-line surface. Kept separate so it can be unit-tested.</summary>
public static class Commands
{
    public static RootCommand Create()
    {
        Option<DirectoryInfo> cdacRootOption = new Option<DirectoryInfo>("--cdac-root", "-c")
        {
            Description = "The cDAC source root (src/native/managed/cdac). Auto-detected if omitted.",
            DefaultValueFactory = _ => Locator.FindCdacRoot()
                ?? throw new InvalidOperationException("Could not locate the cDAC root; pass --cdac-root."),
        };

        Option<DirectoryInfo> outputOption = new Option<DirectoryInfo>("--output", "-o")
        {
            Description = "Directory to write the report artifacts into. Defaults to the tool's output/ folder.",
            DefaultValueFactory = _ => Locator.DefaultOutputDirectory(),
        };

        RootCommand root = new RootCommand("Extract the cDAC contract -> Data usage graph.")
        {
            cdacRootOption,
            outputOption,
        };

        root.SetAction(parseResult =>
        {
            AnalysisOptions options = new AnalysisOptions(
                parseResult.GetValue(cdacRootOption)!,
                parseResult.GetValue(outputOption)!);
            return new AnalysisPipeline(options).Run();
        });

        return root;
    }
}
