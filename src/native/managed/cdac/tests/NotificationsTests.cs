// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

using System;
using System.Collections.Generic;
using Microsoft.Diagnostics.DataContractReader.Contracts;
using Xunit;

namespace Microsoft.Diagnostics.DataContractReader.Tests;

public class NotificationsTests
{
    private const ulong ModuleLoad = 1;
    private const ulong ModuleUnload = 2;
    private const ulong Exception = 5;
    private const ulong Gc = 6;
    private const ulong ExceptionCatcherEnter = 7;
    private const ulong Jit2 = 8;

    private static INotifications CreateContract()
    {
        var target = new TestPlaceholderTarget.Builder(new MockTarget.Architecture { IsLittleEndian = true, Is64Bit = true })
            .UseReader((_, _) => -1)
            .AddContract<INotifications>(version: "c1")
            .Build();
        return target.Contracts.Notifications;
    }

    private static ReadOnlySpan<TargetPointer> MakeExInfo(params ulong[] values)
    {
        TargetPointer[] arr = new TargetPointer[values.Length];
        for (int i = 0; i < values.Length; i++)
            arr[i] = new TargetPointer(values[i]);
        return arr;
    }

    [Theory]
    [InlineData(0ul)]
    [InlineData(3ul)] // JIT_NOTIFICATION (legacy, not handled)
    [InlineData(99ul)]
    public void TryParseNotification_UnknownType_ReturnsFalse(ulong rawType)
    {
        INotifications contract = CreateContract();
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(rawType);
        Assert.False(contract.TryParseNotification(exInfo, out NotificationData? notification));
        Assert.Null(notification);
    }

    [Fact]
    public void TryParseNotification_EmptySpan_ReturnsFalse()
    {
        INotifications contract = CreateContract();
        Assert.False(contract.TryParseNotification(ReadOnlySpan<TargetPointer>.Empty, out NotificationData? notification));
        Assert.Null(notification);
    }

    [Fact]
    public void TryParseNotification_ModuleLoad_ReturnsModuleLoadData()
    {
        INotifications contract = CreateContract();
        ulong expectedModule = 0x1234_5678_9ABC_DEF0ul;
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(ModuleLoad, expectedModule);

        Assert.True(contract.TryParseNotification(exInfo, out NotificationData? notification));

        ModuleLoadNotificationData moduleLoad = Assert.IsType<ModuleLoadNotificationData>(notification);
        Assert.Equal(NotificationType.ModuleLoad, moduleLoad.Type);
        Assert.Equal(expectedModule, moduleLoad.ModuleAddress.Value);
    }

    [Fact]
    public void TryParseNotification_ModuleUnload_ReturnsModuleUnloadData()
    {
        INotifications contract = CreateContract();
        ulong expectedModule = 0xDEAD_BEEF_0000_0001ul;
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(ModuleUnload, expectedModule);

        Assert.True(contract.TryParseNotification(exInfo, out NotificationData? notification));

        ModuleUnloadNotificationData moduleUnload = Assert.IsType<ModuleUnloadNotificationData>(notification);
        Assert.Equal(NotificationType.ModuleUnload, moduleUnload.Type);
        Assert.Equal(expectedModule, moduleUnload.ModuleAddress.Value);
    }

    [Fact]
    public void TryParseNotification_Jit_ReturnsJitData()
    {
        INotifications contract = CreateContract();
        ulong expectedMethodDesc = 0x0000_1111_2222_3333ul;
        ulong expectedNativeCode = 0x0000_4444_5555_6666ul;
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(Jit2, expectedMethodDesc, expectedNativeCode);

        Assert.True(contract.TryParseNotification(exInfo, out NotificationData? notification));

        JitNotificationData jit = Assert.IsType<JitNotificationData>(notification);
        Assert.Equal(NotificationType.Jit2, jit.Type);
        Assert.Equal(expectedMethodDesc, jit.MethodDescAddress.Value);
        Assert.Equal(expectedNativeCode, jit.NativeCodeAddress.Value);
    }

    [Fact]
    public void TryParseNotification_Exception_ReturnsExceptionData()
    {
        INotifications contract = CreateContract();
        ulong expectedThread = 0x0000_CAFE_BABE_0000ul;
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(Exception, expectedThread);

        Assert.True(contract.TryParseNotification(exInfo, out NotificationData? notification));

        ExceptionNotificationData exception = Assert.IsType<ExceptionNotificationData>(notification);
        Assert.Equal(NotificationType.Exception, exception.Type);
        Assert.Equal(expectedThread, exception.ThreadAddress.Value);
    }

    [Fact]
    public void TryParseNotification_Gc_SupportedEvent_ReturnsGcData()
    {
        INotifications contract = CreateContract();
        ulong gcMarkEndType = (ulong)GcEventType.MarkEnd;
        int condemnedGeneration = 2;
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(Gc, gcMarkEndType, (ulong)condemnedGeneration);

        Assert.True(contract.TryParseNotification(exInfo, out NotificationData? notification));

        GcNotificationData gc = Assert.IsType<GcNotificationData>(notification);
        Assert.Equal(NotificationType.Gc, gc.Type);
        Assert.True(gc.IsSupportedEvent);
        Assert.Equal(GcEventType.MarkEnd, gc.EventData.EventType);
        Assert.Equal(condemnedGeneration, gc.EventData.CondemnedGeneration);
    }

    [Fact]
    public void TryParseNotification_Gc_UnsupportedEvent_ReturnsGcDataWithFalseSupported()
    {
        INotifications contract = CreateContract();
        ulong unsupportedGcEventType = (ulong)GcEventType.MarkEnd + 1;
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(Gc, unsupportedGcEventType, 0);

        Assert.True(contract.TryParseNotification(exInfo, out NotificationData? notification));

        GcNotificationData gc = Assert.IsType<GcNotificationData>(notification);
        Assert.False(gc.IsSupportedEvent);
    }

    [Fact]
    public void TryParseNotification_ExceptionCatcherEnter_ReturnsData()
    {
        INotifications contract = CreateContract();
        ulong expectedMethodDesc = 0x0000_AAAA_BBBB_CCCCul;
        uint expectedOffset = 0x42;
        ReadOnlySpan<TargetPointer> exInfo = MakeExInfo(ExceptionCatcherEnter, expectedMethodDesc, expectedOffset);

        Assert.True(contract.TryParseNotification(exInfo, out NotificationData? notification));

        ExceptionCatcherEnterNotificationData catcherEnter = Assert.IsType<ExceptionCatcherEnterNotificationData>(notification);
        Assert.Equal(NotificationType.ExceptionCatcherEnter, catcherEnter.Type);
        Assert.Equal(expectedMethodDesc, catcherEnter.MethodDescAddress.Value);
        Assert.Equal(expectedOffset, catcherEnter.NativeOffset);
    }

    // --- JIT Notification Table Tests ---

    private const uint CLRDATA_METHNOTIFY_NONE = 0;
    private const uint CLRDATA_METHNOTIFY_GENERATED = 1;
    private const uint CLRDATA_METHNOTIFY_DISCARDED = 2;

    // JITNotification struct layout for 64-bit LE:
    //   offset 0: state (ushort, 2 bytes)
    //   offset 2: 6 bytes padding
    //   offset 8: clrModule (ulong, 8 bytes)
    //   offset 16: methodToken (uint, 4 bytes)
    //   offset 20: 4 bytes padding
    //   total size: 24 bytes
    private const int StateOffset = 0;
    private const int ClrModuleOffset = 8;
    private const int MethodTokenOffset = 16;
    private const int EntrySize = 24;
    private const uint TableCapacity = 10;
    private const ulong TableAddress = 0x1_0000;
    private const ulong TablePointerAddress = 0x2_0000;

    private static INotifications CreateContractWithJITTable()
    {
        var arch = new MockTarget.Architecture { IsLittleEndian = true, Is64Bit = true };
        var helpers = new TargetTestHelpers(arch);

        // Build JITNotification type info
        var typeFields = new Dictionary<string, Target.FieldInfo>
        {
            [nameof(Data.JITNotification.State)] = new Target.FieldInfo { Offset = StateOffset },
            [nameof(Data.JITNotification.ClrModule)] = new Target.FieldInfo { Offset = ClrModuleOffset },
            [nameof(Data.JITNotification.MethodToken)] = new Target.FieldInfo { Offset = MethodTokenOffset },
        };
        var types = new Dictionary<DataType, Target.TypeInfo>
        {
            [DataType.JITNotification] = new Target.TypeInfo { Fields = typeFields, Size = EntrySize },
        };

        // Allocate table memory: (TableCapacity + 1) entries for bookkeeping + actual entries
        int totalTableSize = EntrySize * ((int)TableCapacity + 1);
        byte[] tableData = new byte[totalTableSize];

        // Initialize bookkeeping at index 0: length=0, capacity=TableCapacity
        helpers.Write(tableData.AsSpan(MethodTokenOffset), (uint)0); // length = 0
        helpers.WritePointer(tableData.AsSpan(ClrModuleOffset), TableCapacity); // capacity

        // Allocate the pointer to the table
        byte[] tablePointerData = new byte[8];
        helpers.WritePointer(tablePointerData.AsSpan(), TableAddress);

        var builder = new TestPlaceholderTarget.Builder(arch);
        builder.MemoryBuilder.AddHeapFragment(new MockMemorySpace.HeapFragment
        {
            Address = TableAddress,
            Data = tableData,
            Name = "JITNotificationTable"
        });
        builder.MemoryBuilder.AddHeapFragment(new MockMemorySpace.HeapFragment
        {
            Address = TablePointerAddress,
            Data = tablePointerData,
            Name = "JITNotificationTablePointer"
        });

        builder.AddTypes(types);
        builder.AddGlobals(
            (Constants.Globals.JITNotificationTable, TablePointerAddress),
            (Constants.Globals.JITNotificationTableSize, TableCapacity)
        );
        builder.AddContract<INotifications>(version: "c1");

        var target = builder.Build();
        return target.Contracts.Notifications;
    }

    [Fact]
    public void SetCodeNotification_NewEntry_CanBeRead()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module = new(0xAABB_CCDD);
        uint token = 0x0600_0001;

        contract.SetCodeNotification(module, token, CLRDATA_METHNOTIFY_GENERATED);

        uint result = contract.GetCodeNotification(module, token);
        Assert.Equal(CLRDATA_METHNOTIFY_GENERATED, result);
    }

    [Fact]
    public void GetCodeNotification_NotFound_ReturnsNone()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module = new(0xDEAD);
        uint token = 0x0600_9999;

        uint result = contract.GetCodeNotification(module, token);
        Assert.Equal(CLRDATA_METHNOTIFY_NONE, result);
    }

    [Fact]
    public void SetCodeNotification_Update_ChangesFlags()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module = new(0x1234);
        uint token = 0x0600_0001;

        contract.SetCodeNotification(module, token, CLRDATA_METHNOTIFY_GENERATED);
        Assert.Equal(CLRDATA_METHNOTIFY_GENERATED, contract.GetCodeNotification(module, token));

        contract.SetCodeNotification(module, token, CLRDATA_METHNOTIFY_DISCARDED);
        Assert.Equal(CLRDATA_METHNOTIFY_DISCARDED, contract.GetCodeNotification(module, token));
    }

    [Fact]
    public void SetCodeNotification_ClearEntry_ReturnsNone()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module = new(0x1234);
        uint token = 0x0600_0001;

        contract.SetCodeNotification(module, token, CLRDATA_METHNOTIFY_GENERATED);
        Assert.Equal(CLRDATA_METHNOTIFY_GENERATED, contract.GetCodeNotification(module, token));

        contract.SetCodeNotification(module, token, CLRDATA_METHNOTIFY_NONE);
        Assert.Equal(CLRDATA_METHNOTIFY_NONE, contract.GetCodeNotification(module, token));
    }

    [Fact]
    public void SetCodeNotification_MultipleEntries_IndependentlyReadable()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module1 = new(0x1000);
        TargetPointer module2 = new(0x2000);
        uint token1 = 0x0600_0001;
        uint token2 = 0x0600_0002;

        contract.SetCodeNotification(module1, token1, CLRDATA_METHNOTIFY_GENERATED);
        contract.SetCodeNotification(module2, token2, CLRDATA_METHNOTIFY_DISCARDED);

        Assert.Equal(CLRDATA_METHNOTIFY_GENERATED, contract.GetCodeNotification(module1, token1));
        Assert.Equal(CLRDATA_METHNOTIFY_DISCARDED, contract.GetCodeNotification(module2, token2));
    }

    [Fact]
    public void SetCodeNotification_InvalidFlags_Throws()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module = new(0x1234);
        uint token = 0x0600_0001;

        Assert.Throws<ArgumentException>(() =>
            contract.SetCodeNotification(module, token, 0x4));
    }

    [Fact]
    public void SetAllCodeNotifications_ClearsAllEntries()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module = new(0x1000);
        uint token1 = 0x0600_0001;
        uint token2 = 0x0600_0002;

        contract.SetCodeNotification(module, token1, CLRDATA_METHNOTIFY_GENERATED);
        contract.SetCodeNotification(module, token2, CLRDATA_METHNOTIFY_GENERATED);

        contract.SetAllCodeNotifications(TargetPointer.Null, CLRDATA_METHNOTIFY_NONE);

        Assert.Equal(CLRDATA_METHNOTIFY_NONE, contract.GetCodeNotification(module, token1));
        Assert.Equal(CLRDATA_METHNOTIFY_NONE, contract.GetCodeNotification(module, token2));
    }

    [Fact]
    public void SetAllCodeNotifications_FilterByModule_ClearsOnlyMatchingEntries()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module1 = new(0x1000);
        TargetPointer module2 = new(0x2000);
        uint token = 0x0600_0001;

        contract.SetCodeNotification(module1, token, CLRDATA_METHNOTIFY_GENERATED);
        contract.SetCodeNotification(module2, token, CLRDATA_METHNOTIFY_GENERATED);

        contract.SetAllCodeNotifications(module1, CLRDATA_METHNOTIFY_NONE);

        Assert.Equal(CLRDATA_METHNOTIFY_NONE, contract.GetCodeNotification(module1, token));
        Assert.Equal(CLRDATA_METHNOTIFY_GENERATED, contract.GetCodeNotification(module2, token));
    }

    [Fact]
    public void SetAllCodeNotifications_UpdateFlags_ChangesAllMatching()
    {
        INotifications contract = CreateContractWithJITTable();
        TargetPointer module = new(0x1000);
        uint token1 = 0x0600_0001;
        uint token2 = 0x0600_0002;

        contract.SetCodeNotification(module, token1, CLRDATA_METHNOTIFY_GENERATED);
        contract.SetCodeNotification(module, token2, CLRDATA_METHNOTIFY_GENERATED);

        contract.SetAllCodeNotifications(TargetPointer.Null, CLRDATA_METHNOTIFY_DISCARDED);

        Assert.Equal(CLRDATA_METHNOTIFY_DISCARDED, contract.GetCodeNotification(module, token1));
        Assert.Equal(CLRDATA_METHNOTIFY_DISCARDED, contract.GetCodeNotification(module, token2));
    }
}
