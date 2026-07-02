// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Diagnostics;
using Internal.JitInterface;
using static Internal.JitInterface.SYSTEMV_AMD64_CORINFO_STRUCT_REG_PASSING_DESCRIPTOR;
using static Internal.JitInterface.SystemVClassificationType;

namespace Microsoft.Diagnostics.DataContractReader.Contracts.StackWalkHelpers;

// SystemV-AMD64 struct-in-register classifier for the cDAC. Mirrors
// MethodTable::ClassifyEightBytes / ClassifyEightBytesWithManagedLayout in
// src/coreclr/vm/methodtable.cpp, with the same algorithm shape as the
// crossgen2 port in src/coreclr/tools/Common/JitInterface/SystemVStructClassificator.cs.
//
// Structured as a standalone helper (not on the IRuntimeTypeSystem contract)
// so the JitInterface descriptor type stays out of the cDAC's abstractions
// layer. Consumers (CdacTypeHandle) call TryClassify with a Target and a
// TypeHandle; the classifier reaches contract APIs through target.Contracts.
internal static class SystemVStructClassifier
{
    public static bool TryClassify(Target target, TypeHandle typeHandle, out SYSTEMV_AMD64_CORINFO_STRUCT_REG_PASSING_DESCRIPTOR descriptor)
    {
        descriptor = default;
        descriptor.passedInRegisters = false;

        if (!IsSystemVAmd64Target(target))
            return false;

        if (typeHandle.IsNull)
            return false;

        IRuntimeTypeSystem rts = target.Contracts.RuntimeTypeSystem;

        if (!rts.IsValueType(typeHandle))
            return false;

        int typeSize = (int)rts.GetNumInstanceFieldBytes(typeHandle);
        if (typeSize > CLR_SYSTEMV_MAX_STRUCT_BYTES_TO_PASS_IN_REGISTERS)
            return false;

        // Reject intrinsic SIMD Vector types and Int128/UInt128: they are
        // handled specially by the JIT and are not passed as struct registers.
        // See MethodTable::ClassifyEightBytesWithManagedLayout in class.cpp.
        if (IsRejectedIntrinsic(target, typeHandle))
            return false;

        Helper helper = Helper.Create(typeSize);
        if (!ClassifyEightBytes(target, typeHandle, ref helper, startOffsetOfStruct: 0))
            return false;

        descriptor.passedInRegisters = true;
        descriptor.eightByteCount = (byte)helper.EightByteCount;
        descriptor.eightByteClassifications0 = helper.EightByteClassifications[0];
        descriptor.eightByteSizes0 = (byte)helper.EightByteSizes[0];
        descriptor.eightByteOffsets0 = (byte)helper.EightByteOffsets[0];
        descriptor.eightByteClassifications1 = helper.EightByteClassifications[1];
        descriptor.eightByteSizes1 = (byte)helper.EightByteSizes[1];
        descriptor.eightByteOffsets1 = (byte)helper.EightByteOffsets[1];
        return true;
    }

    private static bool IsSystemVAmd64Target(Target target)
    {
        IRuntimeInfo runtimeInfo = target.Contracts.RuntimeInfo;
        return runtimeInfo.GetTargetArchitecture() == RuntimeInfoArchitecture.X64
            && runtimeInfo.GetTargetOperatingSystem() is RuntimeInfoOperatingSystem.Unix or RuntimeInfoOperatingSystem.Apple;
    }

    // True for the type wrappers the JIT never passes as struct-in-registers:
    //   * SIMD wrappers (Vector64/128/256/512, System.Numerics.Vector<T>)
    //   * Int128 / UInt128 (ABI __int128 primitives)
    private static bool IsRejectedIntrinsic(Target target, TypeHandle typeHandle)
        => target.Contracts.RuntimeTypeSystem.GetIntrinsicKind(typeHandle) != IntrinsicTypeKind.None;

    // Maps a field's CorElementType to a SystemV classification. Struct means
    // "recurse into the value type's fields".
    private static SystemVClassificationType ClassifyCorElementType(CorElementType t)
    {
        switch (t)
        {
            case CorElementType.Boolean:
            case CorElementType.Char:
            case CorElementType.I1:
            case CorElementType.U1:
            case CorElementType.I2:
            case CorElementType.U2:
            case CorElementType.I4:
            case CorElementType.U4:
            case CorElementType.I8:
            case CorElementType.U8:
            case CorElementType.I:
            case CorElementType.U:
            case CorElementType.Ptr:
            case CorElementType.FnPtr:
                return SystemVClassificationTypeInteger;
            case CorElementType.R4:
            case CorElementType.R8:
                return SystemVClassificationTypeSSE;
            case CorElementType.Class:
            case CorElementType.Object:
            case CorElementType.String:
            case CorElementType.Array:
            case CorElementType.SzArray:
                return SystemVClassificationTypeIntegerReference;
            case CorElementType.Byref:
                return SystemVClassificationTypeIntegerByRef;
            case CorElementType.ValueType:
                return SystemVClassificationTypeStruct;
            default:
                return SystemVClassificationTypeUnknown;
        }
    }

    // Merges a new classification into an existing one for two fields at the
    // same offset (a union). Mirrors ReClassifyField in SystemVStructClassificator.cs.
    private static SystemVClassificationType ReClassifyField(
        SystemVClassificationType original,
        SystemVClassificationType newField)
    {
        switch (newField)
        {
            case SystemVClassificationTypeInteger:
                // Integer overrides SSE; cannot merge with GC references because
                // language rules prevent an Integer field from overlapping a
                // managed reference at the same offset. If this fires we have
                // classified a bad struct layout upstream.
                Debug.Assert(original is SystemVClassificationTypeInteger or SystemVClassificationTypeSSE);
                return SystemVClassificationTypeInteger;
            case SystemVClassificationTypeSSE:
                Debug.Assert(original is SystemVClassificationTypeInteger or SystemVClassificationTypeSSE);
                return original == SystemVClassificationTypeSSE
                    ? SystemVClassificationTypeSSE
                    : SystemVClassificationTypeInteger;
            case SystemVClassificationTypeIntegerReference:
                Debug.Assert(original == SystemVClassificationTypeIntegerReference);
                return SystemVClassificationTypeIntegerReference;
            case SystemVClassificationTypeIntegerByRef:
                Debug.Assert(original == SystemVClassificationTypeIntegerByRef);
                return SystemVClassificationTypeIntegerByRef;
            default:
                return SystemVClassificationTypeUnknown;
        }
    }

    // Recursive per-struct field walker. Returns false if the struct cannot
    // be classified as a register-passed aggregate. Mirrors ClassifyEightBytes.
    private static bool ClassifyEightBytes(Target target, TypeHandle typeHandle, ref Helper helper, int startOffsetOfStruct)
    {
        IRuntimeTypeSystem rts = target.Contracts.RuntimeTypeSystem;

        int numInstanceFields = 0;
        foreach (TargetPointer fd in rts.GetFieldDescList(typeHandle))
        {
            if (rts.IsFieldDescStatic(fd))
                continue;
            numInstanceFields++;
        }

        if (numInstanceFields == 0)
        {
            helper.LargestFieldOffset = startOffsetOfStruct;
            AssignClassifiedEightByteTypes(ref helper);
            return true;
        }

        // Reject intrinsic Vector wrappers and Int128 at every recursion level:
        // an outer struct containing a Vector128<T> field must also fail.
        // Mirrors MethodTable::ClassifyEightBytesWithManagedLayout.
        if (IsRejectedIntrinsic(target, typeHandle))
            return false;

        foreach (TargetPointer fd in rts.GetFieldDescList(typeHandle))
        {
            if (rts.IsFieldDescStatic(fd))
                continue;

            int fieldOffset = (int)rts.GetFieldDescOffset(fd, fieldDef: null);
            int normalizedFieldOffset = fieldOffset + startOffsetOfStruct;

            CorElementType ft = rts.GetFieldDescType(fd);
            SystemVClassificationType fieldClass = ClassifyCorElementType(ft);

            if (fieldClass == SystemVClassificationTypeStruct)
            {
                TypeHandle nested = rts.GetFieldDescApproxTypeHandle(fd);
                if (nested.IsNull)
                    return false;

                bool wasInEmbeddedStruct = helper.InEmbeddedStruct;
                helper.InEmbeddedStruct = true;
                bool structOk = ClassifyEightBytes(target, nested, ref helper, normalizedFieldOffset);
                helper.InEmbeddedStruct = wasInEmbeddedStruct;
                if (!structOk)
                    return false;

                continue;
            }

            int fieldSize = FieldSizeFromElementType(ft);
            if (fieldSize == 0)
                return false;

            if (normalizedFieldOffset + fieldSize > helper.StructSize)
                return false;

            if ((normalizedFieldOffset % fieldSize) != 0)
                return false;

            if (normalizedFieldOffset <= helper.LargestFieldOffset)
            {
                int i;
                for (i = helper.CurrentUniqueOffsetField - 1; i >= 0; i--)
                {
                    if (helper.FieldOffsets[i] == normalizedFieldOffset)
                    {
                        if (fieldSize > helper.FieldSizes[i])
                            helper.FieldSizes[i] = fieldSize;
                        helper.FieldClassifications[i] = ReClassifyField(helper.FieldClassifications[i], fieldClass);
                        break;
                    }
                }

                if (i >= 0)
                    continue;
            }
            else
            {
                helper.LargestFieldOffset = normalizedFieldOffset;
            }

            if (helper.CurrentUniqueOffsetField >= SYSTEMV_MAX_NUM_FIELDS_IN_REGISTER_PASSED_STRUCT)
                return false;

            Debug.Assert(fieldClass is SystemVClassificationTypeInteger
                or SystemVClassificationTypeIntegerReference
                or SystemVClassificationTypeIntegerByRef
                or SystemVClassificationTypeSSE);

            helper.FieldClassifications[helper.CurrentUniqueOffsetField] = fieldClass;
            helper.FieldSizes[helper.CurrentUniqueOffsetField] = fieldSize;
            helper.FieldOffsets[helper.CurrentUniqueOffsetField] = normalizedFieldOffset;
            helper.CurrentUniqueOffsetField++;
        }

        AssignClassifiedEightByteTypes(ref helper);
        return true;
    }

    private static int FieldSizeFromElementType(CorElementType t)
    {
        switch (t)
        {
            case CorElementType.Boolean:
            case CorElementType.I1:
            case CorElementType.U1:
                return 1;
            case CorElementType.Char:
            case CorElementType.I2:
            case CorElementType.U2:
                return 2;
            case CorElementType.I4:
            case CorElementType.U4:
            case CorElementType.R4:
                return 4;
            case CorElementType.I8:
            case CorElementType.U8:
            case CorElementType.R8:
                return 8;
            case CorElementType.I:
            case CorElementType.U:
            case CorElementType.Ptr:
            case CorElementType.FnPtr:
            case CorElementType.Class:
            case CorElementType.Object:
            case CorElementType.String:
            case CorElementType.Array:
            case CorElementType.SzArray:
            case CorElementType.Byref:
                return 8;
            default:
                return 0;
        }
    }

    // Second pass: walks the struct byte-by-byte from 0..StructSize and
    // assigns unique-offset fields (plus their padding) to eightbytes.
    // Mirrors AssignClassifiedEightByteTypes in SystemVStructClassificator.cs.
    private static void AssignClassifiedEightByteTypes(ref Helper helper)
    {
        const int MaxBytes = CLR_SYSTEMV_MAX_EIGHTBYTES_COUNT_TO_PASS_IN_REGISTERS * SYSTEMV_EIGHT_BYTE_SIZE_IN_BYTES;
        // MaxBytes and SYSTEMV_MAX_NUM_FIELDS_IN_REGISTER_PASSED_STRUCT are
        // both 16 by convention; we just need the field-arrays to cover every
        // possible unique offset.
        Debug.Assert(SYSTEMV_MAX_NUM_FIELDS_IN_REGISTER_PASSED_STRUCT >= MaxBytes);

        if (helper.InEmbeddedStruct)
            return;

        int largestFieldOffset = helper.LargestFieldOffset;
        Debug.Assert(largestFieldOffset != -1);

        // MaxBytes is 16 on SysV AMD64. Stackalloc avoids a per-classification
        // heap allocation on what is otherwise a hot GC-scan path.
        Span<int> sortedFieldOrder = stackalloc int[MaxBytes];
        for (int i = 0; i < MaxBytes; i++)
            sortedFieldOrder[i] = -1;

        int numFields = helper.CurrentUniqueOffsetField;
        for (int i = 0; i < numFields; i++)
        {
            Debug.Assert(helper.FieldOffsets[i] < MaxBytes);
            Debug.Assert(sortedFieldOrder[helper.FieldOffsets[i]] == -1);
            sortedFieldOrder[helper.FieldOffsets[i]] = i;
        }

        int lastFieldOrdinal = sortedFieldOrder[largestFieldOffset];
        int lastFieldSize = lastFieldOrdinal >= 0 ? helper.FieldSizes[lastFieldOrdinal] : 0;
        int offsetAfterLastFieldByte = largestFieldOffset + lastFieldSize;
        SystemVClassificationType lastFieldClass = lastFieldOrdinal >= 0
            ? helper.FieldClassifications[lastFieldOrdinal]
            : SystemVClassificationTypeNoClass;

        int usedEightBytes = 0;
        int accumulatedSizeForEightBytes = 0;
        for (int offset = 0; offset < helper.StructSize; offset++)
        {
            SystemVClassificationType fieldClass;
            int fieldSize;

            int ordinal = sortedFieldOrder[offset];
            if (ordinal == -1)
            {
                if (offset < accumulatedSizeForEightBytes)
                    continue;

                fieldSize = 1;
                fieldClass = offset < offsetAfterLastFieldByte
                    ? SystemVClassificationTypeNoClass
                    : lastFieldClass;
            }
            else
            {
                fieldSize = helper.FieldSizes[ordinal];
                fieldClass = helper.FieldClassifications[ordinal];
            }

            int fieldStartEightByte = offset / SYSTEMV_EIGHT_BYTE_SIZE_IN_BYTES;
            int fieldEndEightByte = (offset + fieldSize - 1) / SYSTEMV_EIGHT_BYTE_SIZE_IN_BYTES;
            Debug.Assert(fieldEndEightByte < CLR_SYSTEMV_MAX_EIGHTBYTES_COUNT_TO_PASS_IN_REGISTERS);

            usedEightBytes = Math.Max(usedEightBytes, fieldEndEightByte + 1);

            for (int eb = fieldStartEightByte; eb <= fieldEndEightByte; eb++)
            {
                SystemVClassificationType existing = helper.EightByteClassifications[eb];
                if (existing == fieldClass)
                {
                    // No change.
                }
                else if (existing == SystemVClassificationTypeNoClass)
                {
                    helper.EightByteClassifications[eb] = fieldClass;
                }
                else if ((existing == SystemVClassificationTypeInteger && fieldClass == SystemVClassificationTypeIntegerReference)
                    || (existing == SystemVClassificationTypeIntegerReference && fieldClass == SystemVClassificationTypeInteger))
                {
                    helper.EightByteClassifications[eb] = SystemVClassificationTypeIntegerReference;
                }
                else if ((existing == SystemVClassificationTypeInteger && fieldClass == SystemVClassificationTypeIntegerByRef)
                    || (existing == SystemVClassificationTypeIntegerByRef && fieldClass == SystemVClassificationTypeInteger))
                {
                    helper.EightByteClassifications[eb] = SystemVClassificationTypeIntegerByRef;
                }
                else
                {
                    helper.EightByteClassifications[eb] = SystemVClassificationTypeInteger;
                }
            }

            int eightByteEnd = (fieldEndEightByte + 1) * SYSTEMV_EIGHT_BYTE_SIZE_IN_BYTES;
            if (offset + fieldSize == eightByteEnd)
                accumulatedSizeForEightBytes = eightByteEnd;
            else
                accumulatedSizeForEightBytes = Math.Max(accumulatedSizeForEightBytes, offset + fieldSize);
        }

        helper.EightByteCount = usedEightBytes;

        for (int eb = 0; eb < usedEightBytes; eb++)
        {
            helper.EightByteOffsets[eb] = eb * SYSTEMV_EIGHT_BYTE_SIZE_IN_BYTES;
            helper.EightByteSizes[eb] = Math.Min(SYSTEMV_EIGHT_BYTE_SIZE_IN_BYTES, helper.StructSize - helper.EightByteOffsets[eb]);
        }
    }

    // Per-recursion state. Mirrors SystemVStructRegisterPassingHelper in the
    // crossgen2 classifier.
    private struct Helper
    {
        internal int StructSize;
        internal int EightByteCount;
        internal SystemVClassificationType[] EightByteClassifications;
        internal int[] EightByteSizes;
        internal int[] EightByteOffsets;

        internal bool InEmbeddedStruct;
        internal int CurrentUniqueOffsetField;
        internal int LargestFieldOffset;
        internal SystemVClassificationType[] FieldClassifications;
        internal int[] FieldSizes;
        internal int[] FieldOffsets;

        internal static Helper Create(int totalStructSize)
        {
            Helper h = default;
            h.StructSize = totalStructSize;
            h.EightByteCount = 0;
            h.InEmbeddedStruct = false;
            h.CurrentUniqueOffsetField = 0;
            h.LargestFieldOffset = -1;

            h.EightByteClassifications = new SystemVClassificationType[CLR_SYSTEMV_MAX_EIGHTBYTES_COUNT_TO_PASS_IN_REGISTERS];
            h.EightByteSizes = new int[CLR_SYSTEMV_MAX_EIGHTBYTES_COUNT_TO_PASS_IN_REGISTERS];
            h.EightByteOffsets = new int[CLR_SYSTEMV_MAX_EIGHTBYTES_COUNT_TO_PASS_IN_REGISTERS];

            h.FieldClassifications = new SystemVClassificationType[SYSTEMV_MAX_NUM_FIELDS_IN_REGISTER_PASSED_STRUCT];
            h.FieldSizes = new int[SYSTEMV_MAX_NUM_FIELDS_IN_REGISTER_PASSED_STRUCT];
            h.FieldOffsets = new int[SYSTEMV_MAX_NUM_FIELDS_IN_REGISTER_PASSED_STRUCT];

            for (int i = 0; i < CLR_SYSTEMV_MAX_EIGHTBYTES_COUNT_TO_PASS_IN_REGISTERS; i++)
                h.EightByteClassifications[i] = SystemVClassificationTypeNoClass;
            for (int i = 0; i < SYSTEMV_MAX_NUM_FIELDS_IN_REGISTER_PASSED_STRUCT; i++)
                h.FieldClassifications[i] = SystemVClassificationTypeNoClass;

            return h;
        }
    }
}
