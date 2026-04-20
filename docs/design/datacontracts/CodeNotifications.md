# Contract CodeNotifications

This contract provides read/write access to the in-target JIT code notification
allowlist. The runtime consults this table when JIT-compiling or discarding a
method; if the (module, methodToken) pair is present with a non-zero flag set,
the runtime raises a `DEBUG_CODE_NOTIFICATION` event so the debugger/DAC can
observe JIT events for that method.

Unlike the [Notifications](Notifications.md) contract (which only decodes
events raised by the runtime), this contract writes into the target process
and may lazily allocate the notification table when needed.

## APIs of contract

``` csharp
// Set the JIT code notification flags for a specific method.
// Flags are CLRDATA_METHNOTIFY_NONE (0), CLRDATA_METHNOTIFY_GENERATED (1), CLRDATA_METHNOTIFY_DISCARDED (2).
void SetCodeNotification(TargetPointer module, uint methodToken, uint flags);

// Get the JIT code notification flags for a specific method.
uint GetCodeNotification(TargetPointer module, uint methodToken);

// Set notification flags for all methods in a module, or all methods if module is null.
void SetAllCodeNotifications(TargetPointer module, uint flags);

// Return the total capacity of the notification table (excluding the bookkeeping slot).
// Callers use this to reject batch requests that cannot possibly fit.
uint GetCodeNotificationCapacity();
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
  `AllocateMemory` is not available (e.g., when the debugger host does not support `ICLRDataTarget2`),
  a `NotSupportedException` is thrown.

``` csharp
void SetCodeNotification(TargetPointer module, uint methodToken, uint flags)
{
    // Read g_pNotificationTable pointer
    TargetPointer tablePointer = target.ReadPointer(
        target.ReadGlobalPointer("JITNotificationTable"));

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

uint GetCodeNotificationCapacity()
{
    return target.ReadGlobal<uint>("JITNotificationTableSize");
}
```
