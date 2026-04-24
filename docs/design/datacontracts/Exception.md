# Contract Exception

This contract is for getting information about exceptions in the process.

## APIs of contract

```csharp
record struct ExceptionData(
    TargetPointer Message,
    TargetPointer InnerException,
    TargetPointer StackTrace,
    TargetPointer WatsonBuckets,
    TargetPointer StackTraceString,
    TargetPointer RemoteStackTraceString,
    int HResult,
    int XCode);
```

``` csharp
TargetPointer GetNestedExceptionInfo(TargetPointer exceptionInfoAddr, out TargetPointer nextNestedExceptionInfo, out TargetPointer thrownObjectHandle);
ExceptionData GetExceptionData(TargetPointer exceptionAddr);
```

## Version 1

Data descriptors used:
- `ExceptionInfo`

Contracts used:
| Contract Name |
| --- |
| `ManagedTypeLayout` |

## Managed Types

Field offsets for the managed type below are resolved via the
[`ManagedTypeLayout`](ManagedTypeLayout.md) contract.

| Type (namespace, name) | Fields used |
| --- | --- |
| (`System`, `Exception`) | `_message`, `_innerException`, `_stackTrace`, `_watsonBuckets`, `_stackTraceString`, `_remoteStackTraceString`, `_HResult`, `_xcode` |

The fields of the managed `System.Exception` type are read from metadata via the
[`ManagedTypeLayout`](ManagedTypeLayout.md) contract rather than from native data
descriptors.

``` csharp
TargetPointer GetNestedExceptionInfo(TargetPointer exceptionInfoAddr, out TargetPointer nextNestedExceptionInfo, out TargetPointer thrownObjectHandle)
{
    nextNestedExceptionInfo = target.ReadPointer(exceptionInfoAddr + /* ExceptionInfo::PreviousNestedInfo offset*/);
    thrownObjectHandle = target.ReadPointer(exceptionInfoAddr + /* ExceptionInfo::ThrownObject offset */);
    if (thrownObjectHandle == TargetPointer.Null)
    {
        return TargetPointer.Null;
    }
    return target.ReadPointer(thrownObjectHandle);
}

ExceptionData GetExceptionData(TargetPointer exceptionAddr)
{
    return new ExceptionData(
        target.ReadPointer(exceptionAddr + /* Exception::Message offset */),
        target.ReadPointer(exceptionAddr + /* Exception::InnerException offset */),
        target.ReadPointer(exceptionAddr + /* Exception::StackTrace offset */),
        target.ReadPointer(exceptionAddr + /* Exception::WatsonBuckets offset */),
        target.ReadPointer(exceptionAddr + /* Exception::StackTraceString offset */),
        target.ReadPointer(exceptionAddr + /* Exception::RemoteStackTraceString offset */),
        target.Read<int>(exceptionAddr + /* Exception::HResult offset */),
        target.Read<int>(exceptionAddr + /* Exception::XCode offset */),
    );
}

```
