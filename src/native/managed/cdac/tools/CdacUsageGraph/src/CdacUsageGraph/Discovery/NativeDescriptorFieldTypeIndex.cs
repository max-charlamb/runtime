// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

namespace CdacUsageGraph.Discovery;

/// <summary>
/// Native storage types declared by <c>CDAC_TYPE_FIELD</c>. This supplements managed
/// <c>IData</c> property metadata for descriptor fields consumed only through shared helpers.
/// </summary>
internal sealed class NativeDescriptorFieldTypeIndex
{
    private const string FieldMacro = "CDAC_TYPE_FIELD(";

    private readonly Dictionary<(string Descriptor, string Field), string> _types;

    private NativeDescriptorFieldTypeIndex(
        Dictionary<(string Descriptor, string Field), string> types) =>
        _types = types;

    public static NativeDescriptorFieldTypeIndex Empty { get; } = new([]);

    public static NativeDescriptorFieldTypeIndex Load(string cdacRoot)
    {
        string path = Path.GetFullPath(Path.Combine(
            cdacRoot,
            "..",
            "..",
            "..",
            "coreclr",
            "vm",
            "datadescriptor",
            "datadescriptor.inc"));
        if (!File.Exists(path))
            throw new InvalidOperationException($"Could not find the native data descriptor at '{path}'.");

        return Parse(File.ReadLines(path));
    }

    internal static NativeDescriptorFieldTypeIndex Parse(IEnumerable<string> lines)
    {
        Dictionary<(string Descriptor, string Field), string> types = [];
        foreach (string line in lines)
        {
            string trimmed = line.TrimStart();
            if (!trimmed.StartsWith(FieldMacro, StringComparison.Ordinal))
                continue;

            int close = trimmed.LastIndexOf(')');
            if (close < FieldMacro.Length)
                throw new InvalidOperationException($"Malformed {FieldMacro.TrimEnd('(')} declaration: {line}");

            List<string> arguments = SplitArguments(
                trimmed.Substring(FieldMacro.Length, close - FieldMacro.Length));
            if (arguments.Count != 4)
                throw new InvalidOperationException($"Malformed {FieldMacro.TrimEnd('(')} declaration: {line}");

            (string, string) key = (arguments[0], arguments[2]);
            string type = ResolveType(arguments[1]);
            if (types.TryGetValue(key, out string? existing) &&
                !string.Equals(existing, type, StringComparison.Ordinal))
            {
                throw new InvalidOperationException(
                    $"Conflicting native types for descriptor field '{key.Item1}.{key.Item2}': " +
                    $"'{existing}' and '{type}'.");
            }
            types[key] = type;
        }
        return new NativeDescriptorFieldTypeIndex(types);
    }

    public string? GetType(string descriptor, string field) =>
        _types.GetValueOrDefault((descriptor, field));

    private static List<string> SplitArguments(string value)
    {
        List<string> arguments = [];
        int start = 0;
        int depth = 0;
        for (int i = 0; i < value.Length; i++)
        {
            switch (value[i])
            {
                case '(':
                    depth++;
                    break;
                case ')':
                    depth--;
                    break;
                case ',' when depth == 0:
                    arguments.Add(value.Substring(start, i - start).Trim());
                    start = i + 1;
                    break;
            }
        }
        arguments.Add(value.Substring(start).Trim());
        return arguments;
    }

    private static string ResolveType(string token) =>
        token switch
        {
            "T_BOOL" => "uint8",
            "T_INT16" => "int16",
            "T_INT32" => "int32",
            "T_INT64" => "int64",
            "T_NINT" => "nint",
            "T_NUINT" => "nuint",
            "T_POINTER" => "pointer",
            "T_UINT8" => "uint8",
            "T_UINT16" => "uint16",
            "T_UINT32" => "uint32",
            "T_UINT64" => "uint64",
            _ when TryUnwrap(token, "TYPE(", out string typeName) => typeName,
            _ when TryUnwrap(token, "EXTERN_TYPE(", out string externalType) => externalType,
            _ when TryUnwrap(token, "T_ARRAY(", out string element) =>
                $"{ResolveType(element)}[]",
            _ => throw new InvalidOperationException(
                $"Unsupported native data descriptor field type '{token}'."),
        };

    private static bool TryUnwrap(string value, string prefix, out string inner)
    {
        if (value.StartsWith(prefix, StringComparison.Ordinal) &&
            value.EndsWith(')'))
        {
            inner = value.Substring(prefix.Length, value.Length - prefix.Length - 1);
            return true;
        }

        inner = string.Empty;
        return false;
    }
}
