// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

#include "createdump.h"
#include <psapi.h>
#include <vector>

// The Windows SDK (winternl.h) we use doesn't have the necessary field (InheritedFromUniqueProcessId)
typedef struct _PROCESS_BASIC_INFORMATION_ {
    NTSTATUS ExitStatus;
    PPEB PebBaseAddress;
    ULONG_PTR AffinityMask;
    KPRIORITY BasePriority;
    ULONG_PTR UniqueProcessId;
    ULONG_PTR InheritedFromUniqueProcessId;
} PROCESS_BASIC_INFORMATION_;

typedef HRESULT (STDAPICALLTYPE* PFN_CLRDataCreateInstance)(REFIID iid, ICLRDataTarget* target, void** iface);

//
// slim cDAC integration: instead of letting dbghelp's auxiliary provider drive the legacy DAC
// (mscordaccore) to select managed memory for heap dumps, we ask the slim contract-based cDAC
// (mscordaccore_slim) -- the existing NativeAOT-compiled cDAC reader with only the
// ICLRDataEnumMemoryRegions surface, no Legacy layer -- to enumerate the managed regions, and feed
// those to MiniDumpWriteDump via a memory callback. Enabled with DOTNET_DbgUseCdac=1. The slim cDAC
// DLL is loaded from DOTNET_DbgCdacPath if set, otherwise from next to coreclr.dll in the target.
//

// The cDAC obtains the runtime's contract descriptor from the data target via this interface
// (a managed-side cDAC interface with no native header). Declared locally with its known IID so
// the managed reader can QueryInterface for it. Mirror of clrdata.idl uuid 17d5b8c6.
struct __declspec(uuid("17d5b8c6-34a9-407f-af4f-a930201d4e02")) ICLRContractLocator : public IUnknown
{
    virtual HRESULT STDMETHODCALLTYPE GetContractDescriptor(ULONG64* contractAddress) = 0;
};

// Reads 'size' bytes at 'address' from the target process. Returns true only if all bytes were read.
static bool
ReadTargetMemory(HANDLE process, ULONG64 address, void* buffer, SIZE_T size)
{
    SIZE_T read = 0;
    return ReadProcessMemory(process, (LPCVOID)(ULONG_PTR)address, buffer, size, &read) && read == size;
}

// Resolves the RVA of an exported symbol in the target's module at 'moduleBase' by walking the PE
// export directory via ReadProcessMemory. createdump is the same architecture as its target, so the
// build's IMAGE_NT_HEADERS layout matches. Returns the absolute target address, or 0 on failure.
static ULONG64
ResolveExportAddress(HANDLE process, ULONG64 moduleBase, const char* symbolName)
{
    IMAGE_DOS_HEADER dosHeader;
    if (!ReadTargetMemory(process, moduleBase, &dosHeader, sizeof(dosHeader)) ||
        dosHeader.e_magic != IMAGE_DOS_SIGNATURE)
    {
        return 0;
    }

    IMAGE_NT_HEADERS ntHeaders;
    if (!ReadTargetMemory(process, moduleBase + dosHeader.e_lfanew, &ntHeaders, sizeof(ntHeaders)) ||
        ntHeaders.Signature != IMAGE_NT_SIGNATURE)
    {
        return 0;
    }

    IMAGE_DATA_DIRECTORY exportDir = ntHeaders.OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT];
    if (exportDir.VirtualAddress == 0 || exportDir.Size == 0)
    {
        return 0;
    }

    IMAGE_EXPORT_DIRECTORY exports;
    if (!ReadTargetMemory(process, moduleBase + exportDir.VirtualAddress, &exports, sizeof(exports)))
    {
        return 0;
    }

    size_t nameLen = strlen(symbolName);
    for (DWORD i = 0; i < exports.NumberOfNames; i++)
    {
        DWORD nameRva = 0;
        if (!ReadTargetMemory(process, moduleBase + exports.AddressOfNames + i * sizeof(DWORD), &nameRva, sizeof(nameRva)))
        {
            continue;
        }

        char name[256];
        if (!ReadTargetMemory(process, moduleBase + nameRva, name, sizeof(name)))
        {
            continue;
        }
        name[sizeof(name) - 1] = '\0';
        if (strncmp(name, symbolName, nameLen + 1) != 0)
        {
            continue;
        }

        WORD ordinal = 0;
        if (!ReadTargetMemory(process, moduleBase + exports.AddressOfNameOrdinals + i * sizeof(WORD), &ordinal, sizeof(ordinal)))
        {
            return 0;
        }
        DWORD funcRva = 0;
        if (!ReadTargetMemory(process, moduleBase + exports.AddressOfFunctions + ordinal * sizeof(DWORD), &funcRva, sizeof(funcRva)))
        {
            return 0;
        }
        return moduleBase + funcRva;
    }

    return 0;
}

// Minimal ICLRDataTarget over a live target process (ReadProcessMemory + module base lookup). Also
// implements ICLRContractLocator so the managed slim cDAC can fetch the runtime's contract
// descriptor (resolved once from the target coreclr's DotNetRuntimeContractDescriptor export).
class ProcessDataTarget : public ICLRDataTarget, public ICLRContractLocator
{
    LONG m_ref;
    HANDLE m_process;
    ULONG64 m_contractDescriptorAddr;

public:
    ProcessDataTarget(HANDLE process) : m_ref(1), m_process(process), m_contractDescriptorAddr(0) { }

    // Resolves the contract descriptor address from the target's coreclr export. Must succeed
    // before the slim cDAC is created. Returns false if coreclr or the export can't be found.
    bool Initialize()
    {
        ULONG64 coreclrBase = 0;
        if (FAILED(GetImageBase(L"coreclr.dll", &coreclrBase)) || coreclrBase == 0)
        {
            return false;
        }
        m_contractDescriptorAddr = ResolveExportAddress(m_process, coreclrBase, "DotNetRuntimeContractDescriptor");
        return m_contractDescriptorAddr != 0;
    }

    STDMETHOD(QueryInterface)(REFIID riid, void** ppvObject)
    {
        if (ppvObject == nullptr)
        {
            return E_POINTER;
        }
        if (riid == IID_IUnknown || riid == __uuidof(ICLRDataTarget))
        {
            *ppvObject = static_cast<ICLRDataTarget*>(this);
            AddRef();
            return S_OK;
        }
        if (riid == __uuidof(ICLRContractLocator))
        {
            *ppvObject = static_cast<ICLRContractLocator*>(this);
            AddRef();
            return S_OK;
        }
        *ppvObject = nullptr;
        return E_NOINTERFACE;
    }

    STDMETHOD_(ULONG, AddRef)() { return InterlockedIncrement(&m_ref); }
    STDMETHOD_(ULONG, Release)()
    {
        LONG ref = InterlockedDecrement(&m_ref);
        if (ref == 0)
        {
            delete this;
        }
        return ref;
    }

    STDMETHOD(GetContractDescriptor)(ULONG64* contractAddress)
    {
        if (contractAddress == nullptr)
        {
            return E_POINTER;
        }
        *contractAddress = m_contractDescriptorAddr;
        return m_contractDescriptorAddr != 0 ? S_OK : E_FAIL;
    }

    STDMETHOD(GetMachineType)(ULONG32* machine)
    {
#if defined(_M_ARM64)
        *machine = IMAGE_FILE_MACHINE_ARM64;
#elif defined(_M_ARM)
        *machine = IMAGE_FILE_MACHINE_ARMNT;
#elif defined(_M_IX86)
        *machine = IMAGE_FILE_MACHINE_I386;
#else
        *machine = IMAGE_FILE_MACHINE_AMD64;
#endif
        return S_OK;
    }

    STDMETHOD(GetPointerSize)(ULONG32* size)
    {
        *size = sizeof(void*);
        return S_OK;
    }

    STDMETHOD(GetImageBase)(LPCWSTR moduleName, CLRDATA_ADDRESS* baseAddress)
    {
        HMODULE modules[1024];
        DWORD needed = 0;
        if (!EnumProcessModulesEx(m_process, modules, sizeof(modules), &needed, LIST_MODULES_ALL))
        {
            return E_FAIL;
        }
        DWORD count = needed / sizeof(HMODULE);
        if (count > ARRAY_SIZE(modules))
        {
            count = ARRAY_SIZE(modules);
        }
        WCHAR name[MAX_PATH];
        for (DWORD i = 0; i < count; i++)
        {
            if (GetModuleBaseNameW(m_process, modules[i], name, ARRAY_SIZE(name)) > 0 &&
                _wcsicmp(name, moduleName) == 0)
            {
                *baseAddress = (CLRDATA_ADDRESS)(ULONG_PTR)modules[i];
                return S_OK;
            }
        }
        return E_FAIL;
    }

    STDMETHOD(ReadVirtual)(CLRDATA_ADDRESS address, PBYTE buffer, ULONG32 size, ULONG32* done)
    {
        SIZE_T read = 0;
        if (!ReadProcessMemory(m_process, (LPCVOID)(ULONG_PTR)address, buffer, size, &read))
        {
            if (done != nullptr)
            {
                *done = 0;
            }
            return HRESULT_FROM_WIN32(GetLastError());
        }
        if (done != nullptr)
        {
            *done = (ULONG32)read;
        }
        return S_OK;
    }

    STDMETHOD(WriteVirtual)(CLRDATA_ADDRESS, PBYTE, ULONG32, ULONG32*) { return E_NOTIMPL; }
    STDMETHOD(GetTLSValue)(ULONG32, ULONG32, CLRDATA_ADDRESS*) { return E_NOTIMPL; }
    STDMETHOD(SetTLSValue)(ULONG32, ULONG32, CLRDATA_ADDRESS) { return E_NOTIMPL; }
    STDMETHOD(GetCurrentThreadID)(ULONG32*) { return E_NOTIMPL; }
    STDMETHOD(GetThreadContext)(ULONG32, ULONG32, ULONG32, PBYTE) { return E_NOTIMPL; }
    STDMETHOD(SetThreadContext)(ULONG32, ULONG32, PBYTE) { return E_NOTIMPL; }
    STDMETHOD(Request)(ULONG32, ULONG32, BYTE*, ULONG32, BYTE*) { return E_NOTIMPL; }
};

// Collects the [address, size) regions reported by the slim cDAC's EnumMemoryRegions.
class CdacRegionCollector : public ICLRDataEnumMemoryRegionsCallback
{
    LONG m_ref;

public:
    std::vector<MINIDUMP_MEMORY_DESCRIPTOR64> m_regions;

    CdacRegionCollector() : m_ref(1) { }

    STDMETHOD(QueryInterface)(REFIID riid, void** ppvObject)
    {
        if (ppvObject == nullptr)
        {
            return E_POINTER;
        }
        if (riid == IID_IUnknown || riid == __uuidof(ICLRDataEnumMemoryRegionsCallback))
        {
            *ppvObject = static_cast<ICLRDataEnumMemoryRegionsCallback*>(this);
            AddRef();
            return S_OK;
        }
        *ppvObject = nullptr;
        return E_NOINTERFACE;
    }

    STDMETHOD_(ULONG, AddRef)() { return InterlockedIncrement(&m_ref); }
    STDMETHOD_(ULONG, Release)() { return InterlockedDecrement(&m_ref); }

    STDMETHOD(EnumMemoryRegion)(CLRDATA_ADDRESS address, ULONG32 size)
    {
        MINIDUMP_MEMORY_DESCRIPTOR64 region;
        region.StartOfMemoryRange = address;
        region.DataSize = size;
        m_regions.push_back(region);
        return S_OK;
    }
};

struct CdacMemoryCallbackState
{
    const std::vector<MINIDUMP_MEMORY_DESCRIPTOR64>* regions;
    size_t index;
};

// MiniDumpWriteDump memory callback: supplies one slim-cDAC region per MemoryCallback invocation.
static BOOL CALLBACK
CdacMemoryCallback(PVOID param, const PMINIDUMP_CALLBACK_INPUT input, PMINIDUMP_CALLBACK_OUTPUT output)
{
    CdacMemoryCallbackState* state = (CdacMemoryCallbackState*)param;
    if (input->CallbackType == MemoryCallback)
    {
        if (state->index < state->regions->size())
        {
            const MINIDUMP_MEMORY_DESCRIPTOR64& region = (*state->regions)[state->index++];
            output->MemoryBase = region.StartOfMemoryRange;
            output->MemorySize = (ULONG)region.DataSize;
        }
        else
        {
            output->MemoryBase = 0;
            output->MemorySize = 0;
        }
    }
    return TRUE;
}

// Determines the slim cDAC DLL path: DOTNET_DbgCdacPath env var, else next to the target's coreclr.dll.
static bool
GetSlimCdacPath(HANDLE hProcess, std::string& path)
{
    char envPath[MAX_LONGPATH];
    DWORD envLen = GetEnvironmentVariableA("DOTNET_DbgCdacPath", envPath, ARRAY_SIZE(envPath));
    if (envLen > 0 && envLen < ARRAY_SIZE(envPath))
    {
        path.assign(envPath, envLen);
        return true;
    }

    HMODULE modules[1024];
    DWORD needed = 0;
    if (!EnumProcessModulesEx(hProcess, modules, sizeof(modules), &needed, LIST_MODULES_ALL))
    {
        return false;
    }
    DWORD count = needed / sizeof(HMODULE);
    if (count > ARRAY_SIZE(modules))
    {
        count = ARRAY_SIZE(modules);
    }
    char name[MAX_PATH];
    for (DWORD i = 0; i < count; i++)
    {
        if (GetModuleBaseNameA(hProcess, modules[i], name, ARRAY_SIZE(name)) > 0 &&
            _stricmp(name, MAKEDLLNAME_A("coreclr")) == 0)
        {
            char fullPath[MAX_LONGPATH];
            if (GetModuleFileNameExA(hProcess, modules[i], fullPath, ARRAY_SIZE(fullPath)) > 0)
            {
                std::string coreclrPath(fullPath);
                size_t sep = coreclrPath.find_last_of("\\/");
                if (sep != std::string::npos)
                {
                    path.assign(coreclrPath, 0, sep + 1);
                    path.append(MAKEDLLNAME_A("mscordaccore_slim"));
                    return true;
                }
            }
        }
    }
    return false;
}

// Writes a dump for the target process using the slim cDAC for managed-memory selection instead of
// the legacy DAC. 'heapTier' selects the region set: true = heap dump (GC heaps + private R/W
// sweep); false = Normal dump (stack-walk-reachable state only). Returns false if the slim cDAC
// could not be used (the caller falls back to the normal MiniDumpWriteDump path).
static bool
TryCreateDumpWithSlimCdac(HANDLE hProcess, DWORD pid, HANDLE hFile, bool heapTier)
{
    std::string cdacPath;
    if (!GetSlimCdacPath(hProcess, cdacPath))
    {
        printf_error("cdac: could not locate mscordaccore_slim (set DOTNET_DbgCdacPath)\n");
        return false;
    }

    HMODULE cdac = LoadLibraryA(cdacPath.c_str());
    if (cdac == nullptr)
    {
        printf_error("cdac: LoadLibrary(%s) FAILED - %s\n", cdacPath.c_str(), GetLastErrorString().c_str());
        return false;
    }

    PFN_CLRDataCreateInstance pfnCreate = (PFN_CLRDataCreateInstance)GetProcAddress(cdac, "CLRDataCreateInstance");
    if (pfnCreate == nullptr)
    {
        printf_error("cdac: GetProcAddress(CLRDataCreateInstance) FAILED\n");
        return false;
    }

    ReleaseHolder<ProcessDataTarget> dataTarget = new ProcessDataTarget(hProcess);
    if (!dataTarget->Initialize())
    {
        printf_error("cdac: could not resolve DotNetRuntimeContractDescriptor from the target\n");
        return false;
    }

    ReleaseHolder<ICLRDataEnumMemoryRegions> enumRegions;
    HRESULT hr = pfnCreate(__uuidof(ICLRDataEnumMemoryRegions), dataTarget, (void**)&enumRegions);
    if (FAILED(hr) || enumRegions == nullptr)
    {
        printf_error("cdac: CLRDataCreateInstance(ICLRDataEnumMemoryRegions) FAILED (%08x)\n", hr);
        return false;
    }

    CdacRegionCollector collector;
    // miniDumpFlags: MiniDumpWithPrivateReadWriteMemory (0x200) => heap tier (full GC heap +
    // R/W sweep); MiniDumpNormal (0) => Normal tier (stack-walk-reachable state only).
    ULONG32 enumFlags = heapTier ? MiniDumpWithPrivateReadWriteMemory : MiniDumpNormal;
    hr = enumRegions->EnumMemoryRegions(&collector, enumFlags, CLRDATA_ENUM_MEM_DEFAULT);
    if (FAILED(hr))
    {
        printf_error("cdac: EnumMemoryRegions FAILED (%08x)\n", hr);
        return false;
    }
    printf_status("cdac: selected %zu managed region(s) [%s tier]\n",
        collector.m_regions.size(), heapTier ? "heap" : "normal");

    // Heap tier (DAC heap-dump model): let dbghelp sweep all private read/write pages
    // (MiniDumpWithPrivateReadWriteMemory) to capture the GC/loader/handle heaps -- object bytes
    // included. The slim cDAC's memory callback adds the memory the sweep misses (executable
    // JIT/stub RX pages, image-backed contract descriptor). Normal tier: MiniDumpNormal (stacks +
    // module headers); the slim cDAC supplies the stack-walk code + method metadata via the
    // callback, no R/W heap sweep.
    MINIDUMP_TYPE dumpType = heapTier
        ? (MINIDUMP_TYPE)(MiniDumpNormal | MiniDumpWithPrivateReadWriteMemory)
        : MiniDumpNormal;

    CdacMemoryCallbackState state = { &collector.m_regions, 0 };
    MINIDUMP_CALLBACK_INFORMATION callbackInfo = {};
    callbackInfo.CallbackRoutine = &CdacMemoryCallback;
    callbackInfo.CallbackParam = &state;

    if (!MiniDumpWriteDump(hProcess, pid, hFile, dumpType, NULL, NULL, &callbackInfo))
    {
        printf_error("cdac: MiniDumpWriteDump - %s\n", GetLastErrorString().c_str());
        return false;
    }

    return true;
}

//
// The Windows create dump code
//
bool
CreateDump(const CreateDumpOptions& options)
{
    HANDLE hFile = INVALID_HANDLE_VALUE;
    HANDLE hProcess = NULL;
    bool result = false;

    _ASSERTE(options.CreateDump);
    _ASSERTE(!options.CrashReport);

    AStringHolder pszName = new char[MAX_LONGPATH + 1];
    std::string dumpPath;

    // On Windows, createdump is restricted for security reasons to only the .NET process (parent process) that launched createdump
    PROCESS_BASIC_INFORMATION_ processInformation;
    NTSTATUS status = NtQueryInformationProcess(GetCurrentProcess(), PROCESSINFOCLASS::ProcessBasicInformation, &processInformation, sizeof(processInformation), NULL);
    if (status != 0)
    {
        printf_error("Failed to get parent process id status %d\n", status);
        goto exit;
    }
    int pid = (int)processInformation.InheritedFromUniqueProcessId;

    hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, pid);
    if (hProcess == NULL)
    {
        printf_error("Invalid process id '%d' - %s\n", pid, GetLastErrorString().c_str());
        goto exit;
    }
    if (GetModuleBaseNameA(hProcess, NULL, pszName, MAX_LONGPATH) <= 0)
    {
        printf_error("Get process name FAILED - %s\n", GetLastErrorString().c_str());
        goto exit;
    }
    if (!FormatDumpName(dumpPath, options.DumpPathTemplate, pszName, pid))
    {
        goto exit;
    }
    printf_status("Writing %s for process %d to file %s\n", GetDumpTypeString(options.DumpType), pid, dumpPath.c_str());

    hFile = CreateFileA(dumpPath.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE)
    {
        printf_error("Invalid dump path '%s' - %s\n", dumpPath.c_str(), GetLastErrorString().c_str());
        goto exit;
    }

    bool cdacHandled = false;
    {
        char envVal[8];
        DWORD envLen = GetEnvironmentVariableA("DOTNET_DbgUseCdac", envVal, ARRAY_SIZE(envVal));
        bool useCdac = (envLen == 1 && envVal[0] == '1');
        // The slim cDAC selects managed memory for non-full dumps; full dumps already capture everything.
        if (useCdac && options.DumpType != DumpType::Full)
        {
            // Heap tier for withheap dumps; Normal tier for normal/triage (stack-walk-reachable only).
            bool heapTier = (options.DumpType == DumpType::Heap);
            printf_status("cdac: collecting managed memory (DOTNET_DbgUseCdac=1, %s tier)\n",
                heapTier ? "heap" : "normal");
            result = TryCreateDumpWithSlimCdac(hProcess, pid, hFile, heapTier);
            cdacHandled = result;
            if (!result)
            {
                printf_error("cdac: collection failed; falling back to default dump path\n");
            }
        }
    }

    if (!cdacHandled)
    {
        int retryCount = 10;
        // Retry the write dump on ERROR_PARTIAL_COPY
        for (int i = 0; i <= retryCount; i++)
        {
            if (MiniDumpWriteDump(hProcess, pid, hFile, GetMiniDumpType(options.DumpType), NULL, NULL, NULL))
            {
                result = true;
                break;
            }
            else
            {
                int err = GetLastError();
                if (err != ERROR_PARTIAL_COPY || i == retryCount)
                {
                    printf_error("MiniDumpWriteDump - %s\n", GetLastErrorString().c_str());
                    break;
                }
                else
                {
                     printf_error("Retry %d of MiniDumpWriteDump due to - %s\n", i, GetLastErrorString().c_str());
                }
            }
        }
    }

exit:
    if (hProcess != NULL)
    {
        CloseHandle(hProcess);
    }

    if (hFile != INVALID_HANDLE_VALUE)
    {
        CloseHandle(hFile);
    }

    return result;
}

std::string
GetLastErrorString()
{
    DWORD error = GetLastError();
    std::string result;
    LPSTR messageBuffer;
    DWORD length = FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        error,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR)&messageBuffer,
        0,
        NULL);
    if (length > 0)
    {
        result.append(messageBuffer, length);
        LocalFree(messageBuffer);

        // Remove the \r\n at the end of the system message. Assumes that the \r is first.
        size_t found = result.find_last_of('\r');
        if (found != std::string::npos)
        {
            result.erase(found);
        }
        result.append(" ");
    }
    char buffer[64];
    _snprintf_s(buffer, sizeof(buffer), sizeof(buffer), "(%d)", error);
    result.append(buffer);
    return result;
}


typedef DWORD(WINAPI *pfnGetTempPathA)(DWORD nBufferLength, LPSTR  lpBuffer);

static volatile pfnGetTempPathA
g_pfnGetTempPathA = nullptr;


DWORD
GetTempPathWrapper(
    IN DWORD nBufferLength,
    OUT LPSTR lpBuffer)
{
    if (g_pfnGetTempPathA == nullptr)
    {
        HMODULE hKernel32 = LoadLibraryExW(L"kernel32.dll", NULL, LOAD_LIBRARY_SEARCH_SYSTEM32);

        pfnGetTempPathA pLocalGetTempPathA = NULL;
        if (hKernel32 != NULL)
        {
            // store to thread local variable to prevent data race
            pLocalGetTempPathA = (pfnGetTempPathA)::GetProcAddress(hKernel32, "GetTempPath2A");
        }

        if (pLocalGetTempPathA == NULL) // method is only available with Windows 10 Creators Update or later
        {
            g_pfnGetTempPathA = &GetTempPathA;
        }
        else
        {
            g_pfnGetTempPathA = pLocalGetTempPathA;
        }
    }

    return g_pfnGetTempPathA(nBufferLength, lpBuffer);
}