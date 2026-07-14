// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System.Text.Json;

namespace CdacUsageGraph.Docs;

/// <summary>
/// The per-contract "meaning" text for each <c>Type.Field</c> descriptor, plus optional
/// <c>_supplement</c>/<c>_suppress</c> overrides, loaded from
/// <c>docs/design/datacontracts/data-descriptor-meanings.json</c>. The file shape is:
/// <code>
/// {
///   "Thread": { "Thread.Id": "Thread identifier", ... },
///   "_supplement": { "Thread": ["ExtraType.ExtraField"] },
///   "_suppress":   { "Thread": ["FalsePositiveType.Field"] }
/// }
/// </code>
/// </summary>
public sealed class DocDescriptorMeanings
{
    private readonly Dictionary<string, Dictionary<string, string>> _meanings;
    private readonly Dictionary<string, List<string>> _supplement;
    private readonly Dictionary<string, List<string>> _suppress;

    private DocDescriptorMeanings(
        Dictionary<string, Dictionary<string, string>> meanings,
        Dictionary<string, List<string>> supplement,
        Dictionary<string, List<string>> suppress)
    {
        _meanings = meanings;
        _supplement = supplement;
        _suppress = suppress;
    }

    public static DocDescriptorMeanings Empty { get; } = new(new(), new(), new());

    /// <summary>Loads the sidecar; returns <see cref="Empty"/> when the file does not exist.</summary>
    public static DocDescriptorMeanings Load(string path)
    {
        if (!File.Exists(path))
            return Empty;

        Dictionary<string, Dictionary<string, string>> meanings = new(StringComparer.Ordinal);
        Dictionary<string, List<string>> supplement = new(StringComparer.Ordinal);
        Dictionary<string, List<string>> suppress = new(StringComparer.Ordinal);

        using JsonDocument doc = JsonDocument.Parse(File.ReadAllText(path));
        foreach (JsonProperty top in doc.RootElement.EnumerateObject())
        {
            switch (top.Name)
            {
                case "_supplement":
                    ReadListMap(top.Value, supplement);
                    break;
                case "_suppress":
                    ReadListMap(top.Value, suppress);
                    break;
                default:
                    Dictionary<string, string> byKey = new(StringComparer.Ordinal);
                    foreach (JsonProperty entry in top.Value.EnumerateObject())
                    {
                        ValidateDescriptorKey(entry.Name, $"meaning entry for contract '{top.Name}'");
                        byKey[entry.Name] = entry.Value.GetString() ?? string.Empty;
                    }
                    meanings[top.Name] = byKey;
                    break;
            }
        }

        return new DocDescriptorMeanings(meanings, supplement, suppress);
    }

    /// <summary>The meaning for <paramref name="key"/> (<c>Type.Field</c>), or a TODO placeholder.</summary>
    public string Meaning(string contractShort, string key) =>
        _meanings.TryGetValue(contractShort, out Dictionary<string, string>? byKey) && byKey.TryGetValue(key, out string? m)
            ? m
            : "_TODO: describe_";

    public IReadOnlyList<string> Supplement(string contractShort) =>
        _supplement.TryGetValue(contractShort, out List<string>? l) ? l : [];

    public IReadOnlyList<string> Suppress(string contractShort) =>
        _suppress.TryGetValue(contractShort, out List<string>? l) ? l : [];

    private static void ReadListMap(JsonElement obj, Dictionary<string, List<string>> into)
    {
        foreach (JsonProperty p in obj.EnumerateObject())
        {
            List<string> list = new();
            foreach (JsonElement v in p.Value.EnumerateArray())
            {
                if (v.GetString() is string s)
                {
                    ValidateDescriptorKey(s, $"'{p.Name}' override");
                    list.Add(s);
                }
            }
            into[p.Name] = list;
        }
    }

    private static void ValidateDescriptorKey(string key, string context)
    {
        int dot = key.IndexOf('.', StringComparison.Ordinal);
        if (dot <= 0 || dot == key.Length - 1)
            throw new JsonException($"Invalid data-descriptor key '{key}' in {context}; expected 'Type.Field'.");
    }
}
