// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Text.Json;

namespace Microsoft.Diagnostics.DataContractReader;

/// <summary>
///   A parser for the JSON representation of a contract descriptor.
/// </summary>
/// <remarks>
/// <see href="https://github.com/dotnet/runtime/blob/main/docs/design/datacontracts/data_descriptor.md">See design doc</see> for the format.
///
/// The parser is written directly against <see cref="Utf8JsonReader"/> (no <c>JsonSerializer</c> /
/// source-generated <c>JsonSerializerContext</c>) so it is fully trimming- and reflection-free-safe:
/// this lets the reader be compiled with NativeAOT <c>--reflectiondata:none</c>. The
/// <c>JsonSerializerContext</c> path relies on runtime custom-attribute reflection, which is stripped
/// in that mode.
/// </remarks>
public partial class ContractDescriptorParser
{
    // data_descriptor.md uses a distinguished property name to indicate the size of a type
    public const string TypeDescriptorSizeSigil = "!";

    private static readonly JsonReaderOptions s_readerOptions = new()
    {
        AllowTrailingCommas = true,
        CommentHandling = JsonCommentHandling.Skip,
    };

    /// <summary>
    ///  Parses the "compact" representation of a contract descriptor.
    /// </summary>
    public static ContractDescriptor? ParseCompact(ReadOnlySpan<byte> json)
    {
        Utf8JsonReader reader = new(json, s_readerOptions);
        if (!reader.Read())
        {
            throw new JsonException();
        }
        return ParseDescriptor(ref reader);
    }

    // Reads the top-level contract descriptor object. Property names are matched case-sensitively
    // (the compact format is camelCase); unknown top-level members are skipped.
    private static ContractDescriptor ParseDescriptor(ref Utf8JsonReader reader)
    {
        if (reader.TokenType != JsonTokenType.StartObject)
        {
            throw new JsonException();
        }

        ContractDescriptor descriptor = new();
        while (reader.Read())
        {
            if (reader.TokenType == JsonTokenType.EndObject)
            {
                return descriptor;
            }
            if (reader.TokenType != JsonTokenType.PropertyName)
            {
                throw new JsonException();
            }

            string? name = reader.GetString();
            reader.Read(); // advance to the property value
            switch (name)
            {
                case "version":
                    descriptor.Version = ReadNullableInt32(ref reader);
                    break;
                case "baseline":
                    descriptor.Baseline = reader.TokenType == JsonTokenType.Null ? null : reader.GetString();
                    break;
                case "contracts":
                    descriptor.Contracts = ReadStringDictionary(ref reader);
                    break;
                case "types":
                    descriptor.Types = ReadTypeDictionary(ref reader);
                    break;
                case "globals":
                    descriptor.Globals = ReadGlobalDictionary(ref reader);
                    break;
                case "subDescriptors":
                    descriptor.SubDescriptors = ReadGlobalDictionary(ref reader);
                    break;
                default:
                    reader.Skip();
                    break;
            }
        }
        throw new JsonException();
    }

    private static int? ReadNullableInt32(ref Utf8JsonReader reader)
    {
        if (reader.TokenType == JsonTokenType.Null)
        {
            return null;
        }
        if (TryGetInt32FromToken(ref reader, out int value))
        {
            return value;
        }
        throw new JsonException();
    }

    private static Dictionary<string, string> ReadStringDictionary(ref Utf8JsonReader reader)
    {
        if (reader.TokenType != JsonTokenType.StartObject)
        {
            throw new JsonException();
        }
        Dictionary<string, string> result = new();
        while (reader.Read())
        {
            if (reader.TokenType == JsonTokenType.EndObject)
            {
                return result;
            }
            if (reader.TokenType != JsonTokenType.PropertyName)
            {
                throw new JsonException();
            }
            string key = reader.GetString()!;
            reader.Read();
            result[key] = reader.GetString() ?? throw new JsonException();
        }
        throw new JsonException();
    }

    private static Dictionary<string, TypeDescriptor> ReadTypeDictionary(ref Utf8JsonReader reader)
    {
        if (reader.TokenType != JsonTokenType.StartObject)
        {
            throw new JsonException();
        }
        Dictionary<string, TypeDescriptor> result = new();
        while (reader.Read())
        {
            if (reader.TokenType == JsonTokenType.EndObject)
            {
                return result;
            }
            if (reader.TokenType != JsonTokenType.PropertyName)
            {
                throw new JsonException();
            }
            string key = reader.GetString()!;
            reader.Read();
            result[key] = ParseTypeDescriptor(ref reader);
        }
        throw new JsonException();
    }

    private static Dictionary<string, GlobalDescriptor> ReadGlobalDictionary(ref Utf8JsonReader reader)
    {
        if (reader.TokenType != JsonTokenType.StartObject)
        {
            throw new JsonException();
        }
        Dictionary<string, GlobalDescriptor> result = new();
        while (reader.Read())
        {
            if (reader.TokenType == JsonTokenType.EndObject)
            {
                return result;
            }
            if (reader.TokenType != JsonTokenType.PropertyName)
            {
                throw new JsonException();
            }
            string key = reader.GetString()!;
            reader.Read();
            result[key] = ParseGlobalDescriptor(ref reader);
        }
        throw new JsonException();
    }

    public class ContractDescriptor
    {
        public int? Version { get; set; }
        public string? Baseline { get; set; }
        public Dictionary<string, string>? Contracts { get; set; }

        public Dictionary<string, TypeDescriptor>? Types { get; set; }

        public Dictionary<string, GlobalDescriptor>? Globals { get; set; }

        public Dictionary<string, GlobalDescriptor>? SubDescriptors { get; set; }

        // Retained for API/test compatibility. The direct Utf8JsonReader parser skips unknown
        // members rather than collecting them, so this is always null.
        public Dictionary<string, JsonElement>? Extras { get; set; }

        public override string ToString()
        {
            return $"Version: {Version}, Baseline: {Baseline}, Contracts: {Contracts?.Count}, Types: {Types?.Count}, Globals: {Globals?.Count}, SubDescriptors: {SubDescriptors?.Count}";
        }

    }

    public class TypeDescriptor
    {
        public uint? Size { get; set; }
        public Dictionary<string, FieldDescriptor>? Fields { get; set; }
    }

    public class FieldDescriptor
    {
        public string? Type { get; set; }
        public int Offset { get; set; }
    }

    public class GlobalDescriptor
    {
        [MemberNotNullWhen(true, nameof(NumericValue))]
        public bool Indirect { get; set; }
        public string? Type { get; set; }

        // When the descriptor is indirect, NumericValue must be non-null to point to the actual data
        public ulong? NumericValue { get; set; }
        public string? StringValue { get; set; }
    }

    // Almost a normal dictionary parse except:
    //  1. looks for a special key "!" to set the Size property
    //  2. field names are property names, but treated case-sensitively
    private static TypeDescriptor ParseTypeDescriptor(ref Utf8JsonReader reader)
    {
        if (reader.TokenType != JsonTokenType.StartObject)
            throw new JsonException();
        uint? size = null;
        Dictionary<string, FieldDescriptor> fields = new();
        while (reader.Read())
        {
            switch (reader.TokenType)
            {
                case JsonTokenType.EndObject:
                    return new TypeDescriptor { Size = size, Fields = fields };
                case JsonTokenType.PropertyName:
                    string? fieldNameOrSizeSigil = reader.GetString();
                    reader.Read(); // read the next value: either a number or a field descriptor
                    if (fieldNameOrSizeSigil == TypeDescriptorSizeSigil)
                    {
                        uint newSize = reader.GetUInt32();
                        if (size is not null)
                        {
                            throw new JsonException($"Size specified multiple times: {size} and {newSize}");
                        }
                        size = newSize;
                    }
                    else
                    {
                        string? fieldName = fieldNameOrSizeSigil;
                        FieldDescriptor field = ParseFieldDescriptor(ref reader);
                        if (fieldName is null)
                            throw new JsonException();
                        if (!fields.TryAdd(fieldName, field))
                        {
                            throw new JsonException($"Duplicate field name: {fieldName}");
                        }
                    }
                    break;
                default:
                    throw new JsonException();
            }
        }
        throw new JsonException();
    }

    // Compact field descriptors are either a number or a two element array
    // 1. number - no type, offset is given as the number
    // 2. [number, string] - offset is given as the number, type name is given as the string
    private static FieldDescriptor ParseFieldDescriptor(ref Utf8JsonReader reader)
    {
        if (TryGetInt32FromToken(ref reader, out int offset))
            return new FieldDescriptor { Offset = offset };
        if (reader.TokenType != JsonTokenType.StartArray)
            throw new JsonException();
        reader.Read();
        //   [number, string]
        //    ^ we're here
        if (!TryGetInt32FromToken(ref reader, out offset))
            throw new JsonException();
        reader.Read(); // string
        if (reader.TokenType != JsonTokenType.String)
            throw new JsonException();
        string? type = reader.GetString();
        reader.Read(); // end of array
        if (reader.TokenType != JsonTokenType.EndArray)
            throw new JsonException();
        return new FieldDescriptor { Type = type, Offset = offset };
    }

    private static GlobalDescriptor ParseGlobalDescriptor(ref Utf8JsonReader reader)
    {
        // four cases:
        // 1. value - no type, direct value, given value
        // 2. [number] - no type, indirect value, given aux data ptr
        // 3. [value, string] - type, direct value, given value
        // 4. [[number], string] - type, indirect value, given aux data ptr
        // value can either be a string or a number. if a string is able to be parsed as a number, it is read as both

        // Case 1: value
        if (TryGetGlobalValueFromToken(ref reader, out GlobalValue valueCase1))
            return new GlobalDescriptor { NumericValue = valueCase1.NumericValue, StringValue = valueCase1.StringValue, Indirect = false };
        if (reader.TokenType != JsonTokenType.StartArray)
            throw new JsonException();
        reader.Read();
        // we're in case 2, 3, or 4:
        // case 2: [number]
        //          ^ we're here
        // case 3: [value, string]
        //          ^ we're here
        // case 4: [[number], string]
        //          ^ we're here
        if (TryGetGlobalValueFromToken(ref reader, out GlobalValue valueCase2or3))
        {
            // case 2 or 3
            reader.Read(); // end of array (case 2) or string (case 3)
            if (reader.TokenType == JsonTokenType.EndArray) // it was case 2
            {
                if (valueCase2or3.NumericValue is null)
                    throw new JsonException("Indirect global value could not be converted to a number.");
                return new GlobalDescriptor { NumericValue = valueCase2or3.NumericValue, StringValue = valueCase2or3.StringValue, Indirect = true };
            }
            if (reader.TokenType == JsonTokenType.String) // it was case 3
            {
                string? type = reader.GetString();
                reader.Read(); // end of array for case 3
                if (reader.TokenType != JsonTokenType.EndArray)
                    throw new JsonException();
                return new GlobalDescriptor { Type = type, NumericValue = valueCase2or3.NumericValue, StringValue = valueCase2or3.StringValue, Indirect = false };
            }
            throw new JsonException();
        }
        if (reader.TokenType == JsonTokenType.StartArray)
        {
            // case 4: [[number], string]
            //          ^ we're here
            reader.Read(); // number
            if (!TryGetGlobalValueFromToken(ref reader, out GlobalValue valueCase4))
                throw new JsonException();
            reader.Read(); // end of inner array
            if (reader.TokenType != JsonTokenType.EndArray)
                throw new JsonException();
            reader.Read(); // string
            if (reader.TokenType != JsonTokenType.String)
                throw new JsonException();
            string? type = reader.GetString();
            reader.Read(); // end of outer array
            if (reader.TokenType != JsonTokenType.EndArray)
                throw new JsonException();
            if (valueCase4.NumericValue is null)
                throw new JsonException("Indirect global value could not be converted to a number.");
            return new GlobalDescriptor { Type = type, NumericValue = valueCase4.NumericValue, StringValue = valueCase4.StringValue, Indirect = true };
        }
        throw new JsonException();
    }

    private struct GlobalValue
    {
        public ulong? NumericValue;
        public string? StringValue;
    }

    private static bool TryGetGlobalValueFromToken(ref Utf8JsonReader reader, out GlobalValue directGlobalValue)
    {
        bool foundNumeric = TryGetUInt64FromToken(ref reader, out ulong numericValue);
        bool foundString = TryGetStringFromToken(ref reader, out string stringValue);
        if (foundNumeric || foundString)
        {
            // this parsed as a valid direct global value
            directGlobalValue = new GlobalValue
            {
                NumericValue = foundNumeric ? numericValue : null,
                StringValue = foundString ? stringValue : null
            };
            return true;
        }
        directGlobalValue = default;
        return false;
    }

    private static bool TryGetStringFromToken(ref Utf8JsonReader reader, out string value)
    {
        value = string.Empty;
        if (reader.TokenType == JsonTokenType.String && reader.GetString() is string stringValue)
        {
            value = stringValue;
            return true;
        }
        return false;
    }

    // Somewhat flexible parsing of numbers, allowing json number tokens or strings as decimal or hex, possibly negated.
    private static bool TryGetUInt64FromToken(ref Utf8JsonReader reader, out ulong value)
    {
        if (reader.TokenType == JsonTokenType.Number)
        {
            if (reader.TryGetUInt64(out value))
                return true;
            if (reader.TryGetInt64(out long signedValue))
            {
                value = (ulong)signedValue;
                return true;
            }
        }
        if (reader.TokenType == JsonTokenType.String)
        {
            var s = reader.GetString();
            if (s == null)
            {
                value = 0u;
                return false;
            }
            if (ulong.TryParse(s, out value))
                return true;
            if (long.TryParse(s, out long signedValue))
            {
                value = (ulong)signedValue;
                return true;
            }
            if (s.StartsWith("0x", StringComparison.OrdinalIgnoreCase) &&
                ulong.TryParse(s.AsSpan(2), System.Globalization.NumberStyles.HexNumber, null, out value))
            {
                return true;
            }
            if (s.StartsWith("-0x", StringComparison.OrdinalIgnoreCase) &&
                ulong.TryParse(s.AsSpan(3), System.Globalization.NumberStyles.HexNumber, null, out ulong negValue))
            {
                value = ~negValue + 1; // two's complement
                return true;
            }
        }
        value = 0;
        return false;
    }

    // Somewhat flexible parsing of numbers, allowing json number tokens or strings as decimal or hex, possibly negated.
    private static bool TryGetInt32FromToken(ref Utf8JsonReader reader, out int value)
    {
        if (reader.TokenType == JsonTokenType.Number)
        {
            value = reader.GetInt32();
            return true;
        }
        if (reader.TokenType == JsonTokenType.String)
        {
            var s = reader.GetString();
            if (s == null)
            {
                value = 0;
                return false;
            }
            if (int.TryParse(s, out value))
            {
                return true;
            }
            if (s.StartsWith("0x", StringComparison.OrdinalIgnoreCase) &&
                int.TryParse(s.AsSpan(2), System.Globalization.NumberStyles.HexNumber, null, out value))
            {
                return true;
            }
            if (s.StartsWith("-0x", StringComparison.OrdinalIgnoreCase) &&
                int.TryParse(s.AsSpan(3), System.Globalization.NumberStyles.HexNumber, null, out int negValue))
            {
                value = -negValue;
                return true;
            }
        }
        value = 0;
        return false;
    }
}
