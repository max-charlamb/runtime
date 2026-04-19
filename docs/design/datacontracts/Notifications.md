# Contract Notifications

This contract is for debugger notifications.

## APIs of contract

``` csharp
// Set the GC notification for condemned generations
// The argument is a bitmask where the i-th bit set represents the i-th generation.
void SetGcNotification(int condemnedGeneration);

// Parses the exception information array into a typed notification object.
// Returns false if the notification type is unknown. Pattern match on the result to access notification-specific fields.
bool TryParseNotification(ReadOnlySpan<TargetPointer> exceptionInformation, out NotificationData? notification);

// Set the JIT code notification flags for a specific method.
// Flags are CLRDATA_METHNOTIFY_NONE (0), CLRDATA_METHNOTIFY_GENERATED (1), CLRDATA_METHNOTIFY_DISCARDED (2).
void SetCodeNotification(TargetPointer module, uint methodToken, uint flags);

// Get the JIT code notification flags for a specific method.
uint GetCodeNotification(TargetPointer module, uint methodToken);

// Set notification flags for all methods in a module, or all methods if module is null.
void SetAllCodeNotifications(TargetPointer module, uint flags);
```

## Version 1

Data descriptors used:
| Data Descriptor Name | Field | Type | Purpose |
| --- | --- | --- | --- |
| `JITNotification` | `State` | uint16 | Notification flags (CLRDATA_METHNOTIFY_*) |
| `JITNotification` | `ClrModule` | nuint | Target pointer to the module |
| `JITNotification` | `MethodToken` | uint32 | Method metadata token |

Global variables used:
| Global Name | Type | Purpose |
| --- | --- | --- |
| `GcNotificationFlags` | TargetPointer | Global flag for storing GC notification data |
| `JITNotificationTable` | TargetPointer | Pointer to the `g_pNotificationTable` array of `JITNotification` entries |
| `JITNotificationTableSize` | uint32 | Maximum number of entries in the notification table (excluding bookkeeping) |

Contracts used: none

The JIT notification table is an array of `JITNotification` structs. Index 0 is reserved for
bookkeeping: its `MethodToken` field stores the current entry count and its `ClrModule` field stores
the table capacity. Actual entries start at index 1.

On Windows, the table starts as NULL (`g_pNotificationTable == 0`). On Unix, it is pre-allocated
at startup. The contract handles both cases:
- **GetCodeNotification** returns `CLRDATA_METHNOTIFY_NONE` when the table is NULL.
- **SetAllCodeNotifications** is a no-op when the table is NULL.
- **SetCodeNotification** with `CLRDATA_METHNOTIFY_NONE` is a no-op when the table is NULL.
- **SetCodeNotification** with a non-zero flag lazily allocates the table via `Target.AllocateMemory`,
  initializes the bookkeeping entry, and writes the pointer back to `g_pNotificationTable`. If
  `AllocateMemory` is not available (e.g., when the cDAC is loaded via `cdac_reader_init`), a
  `NotSupportedException` is thrown.

``` csharp
public enum GcEventType
{
    MarkEnd = 1,
}

public record struct GcEventData(GcEventType EventType, int CondemnedGeneration);

public enum NotificationType
{
    Unknown = 0,
    ModuleLoad,
    ModuleUnload,
    Jit2,
    Exception,
    Gc,
    ExceptionCatcherEnter,
}

public abstract record NotificationData(NotificationType Type);
public record ModuleLoadNotificationData(TargetPointer ModuleAddress) : NotificationData(NotificationType.ModuleLoad);
public record ModuleUnloadNotificationData(TargetPointer ModuleAddress) : NotificationData(NotificationType.ModuleUnload);
public record JitNotificationData(TargetPointer MethodDescAddress, TargetPointer NativeCodeAddress) : NotificationData(NotificationType.Jit2);
public record ExceptionNotificationData(TargetPointer ThreadAddress) : NotificationData(NotificationType.Exception);
public record GcNotificationData(GcEventData EventData, bool IsSupportedEvent) : NotificationData(NotificationType.Gc);
public record ExceptionCatcherEnterNotificationData(TargetPointer MethodDescAddress, uint NativeOffset) : NotificationData(NotificationType.ExceptionCatcherEnter);

void SetGcNotification(int condemnedGeneration)
{
    TargetPointer pGcNotificationFlags = _target.ReadGlobalPointer("GcNotificationFlags");
    uint currentFlags = _target.Read<uint>(pGcNotificationFlags);
    if (condemnedGeneration == 0)
        _target.Write<uint>(pGcNotificationFlags, 0);
    else
    {
        _target.Write<uint>(pGcNotificationFlags, (uint)(currentFlags | condemnedGeneration));
    }
}

private enum NativeNotificationType : uint
{
    ModuleLoad = 1,
    ModuleUnload = 2,
    Exception = 5,
    Gc = 6,
    ExceptionCatcherEnter = 7,
    Jit2 = 8,
}

bool TryParseNotification(ReadOnlySpan<TargetPointer> exceptionInformation, out NotificationData? notification)
{
    if (exceptionInformation.IsEmpty)
    {
        notification = null;
        return false;
    }

    notification = (NativeNotificationType)(uint)exceptionInformation[0].Value switch
    {
        NativeNotificationType.ModuleLoad => new ModuleLoadNotificationData(exceptionInformation[1]),
        NativeNotificationType.ModuleUnload => new ModuleUnloadNotificationData(exceptionInformation[1]),
        NativeNotificationType.Jit2 => new JitNotificationData(exceptionInformation[1], exceptionInformation[2]),
        NativeNotificationType.Exception => new ExceptionNotificationData(exceptionInformation[1]),
        NativeNotificationType.Gc => ParseGcNotification(exceptionInformation),
        NativeNotificationType.ExceptionCatcherEnter => new ExceptionCatcherEnterNotificationData(exceptionInformation[1], (uint)exceptionInformation[2].Value),
        _ => null,
    };

    return notification is not null;
}

void SetCodeNotification(TargetPointer module, uint methodToken, uint flags)
{
    // Read g_pNotificationTable pointer
    TargetPointer tablePointer = target.ReadPointer(
        target.ReadGlobalPointer("JITNotificationTable"));

    // Handle null table
    if (tablePointer == null)
    {
        if (flags == CLRDATA_METHNOTIFY_NONE) return; // nothing to clear
        // Lazily allocate via Target.AllocateMemory
        tablePointer = AllocateAndInitializeTable();
    }

    // Read bookkeeping from index 0
    uint length = Read<uint>(tablePointer + MethodTokenOffset);
    uint capacity = ReadNUInt(tablePointer + ClrModuleOffset);
    ulong entriesBase = tablePointer + entrySize;

    if (flags == CLRDATA_METHNOTIFY_NONE)
    {
        // Find and clear the matching entry
    }
    else
    {
        // Find existing entry and update, or find free slot and insert
    }
}

uint GetCodeNotification(TargetPointer module, uint methodToken)
{
    // If table pointer is NULL, return CLRDATA_METHNOTIFY_NONE
    // Otherwise read the table and find the matching entry, return its state
    // Returns CLRDATA_METHNOTIFY_NONE if not found
}

void SetAllCodeNotifications(TargetPointer module, uint flags)
{
    // If table pointer is NULL, return (no-op)
    // Iterate all active entries; if module is non-null, filter by module
    // Set or clear each matching entry's flags
}
```
