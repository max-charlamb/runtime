// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.

#ifndef NIBBLEMAPMACROS_H_
#define NIBBLEMAPMACROS_H_

///////////////////////////////////////////////////////////////////////
////   some mmgr stuff for JIT, especially for jit code blocks
///////////////////////////////////////////////////////////////////////
//
// In order to quickly find the start of a jit code block
// we keep track of all those positions via a map.
// Each entry in this map represents 32 byte (a bucket) of the code heap.
// We make the assumption that no two code-blocks can start in
// the same 32byte bucket;
// Additionally we assume that every code header is DWORD aligned.
// Because we cannot guarantee that jitblocks always start at
// multiples of 32 bytes we cannot use a simple bitmap; instead we
// use a nibble (4 bit) per bucket and encode the offset of the header
// inside the bucket (in DWORDS). Each 32-bit DWORD represents 256 bytes
// in the mapped code region. In order to make initialization
// easier we add one to the real offset, a nibble-value of zero
// means that there is no header start in the resp. bucket.
// To have constant time reads, we also store relative pointers
// in DWORDs which represent code regions which are completed
// covered by a function. This is indicated by the nibble value
// in the lowest bits of the DWORD having a value > 8.
// 
// Pointers are encoded in DWORDS by using the top 28 bits as normal.
// The bottom 4 bits are read as a nibble (the value must be greater than 8
// to identify the DWORD as a pointer) which encodes the final 2 bits of
// information. 

#if defined(HOST_64BIT)
// TODO: bump up the windows CODE_ALIGN to 16 and iron out any nibble map bugs that exist.
# define CODE_ALIGN             4
# define LOG2_CODE_ALIGN        2
#else
# define CODE_ALIGN             sizeof(DWORD)                                // 4 byte boundry
# define LOG2_CODE_ALIGN        2
#endif
#define NIBBLE_MASK             0xfu
#define NIBBLE_SIZE             4                                            // 4 bits
#define LOG2_NIBBLE_SIZE        2
#define NIBBLES_PER_DWORD       (2 * sizeof(DWORD))                          // 8 (4-bit) nibbles per dword
#define NIBBLES_PER_DWORD_MASK  (NIBBLES_PER_DWORD - 1)                      // 7
#define LOG2_NIBBLES_PER_DWORD  3
#define BYTES_PER_BUCKET        (NIBBLES_PER_DWORD * CODE_ALIGN)             // 32 bytes per bucket
#define LOG2_BYTES_PER_BUCKET   (LOG2_CODE_ALIGN + LOG2_NIBBLES_PER_DWORD)   //  5 bits per bucket
#define MASK_BYTES_PER_BUCKET   (BYTES_PER_BUCKET - 1)                       // 31
#define BYTES_PER_DWORD         (NIBBLES_PER_DWORD * BYTES_PER_BUCKET)       // 256 bytes per dword
#define LOG2_BYTES_PER_DWORD    (LOG2_NIBBLES_PER_DWORD + LOG2_BYTES_PER_BUCKET) // 8 bits per dword
#define HIGHEST_NIBBLE_BIT      (32 - NIBBLE_SIZE)                           // 28 (i.e 32 - 4)
#define HIGHEST_NIBBLE_MASK     (NIBBLE_MASK << HIGHEST_NIBBLE_BIT)          // 0xf0000000

#define ADDR2POS(x)                      ((x) >> LOG2_BYTES_PER_BUCKET)
#define ADDR2OFFS(x)            (DWORD)  ((((x) & MASK_BYTES_PER_BUCKET) >> LOG2_CODE_ALIGN) + 1)
#define POSOFF2ADDR(pos, of)    (size_t) (((pos) << LOG2_BYTES_PER_BUCKET) + (((of) - 1) << LOG2_CODE_ALIGN))
#define HEAP2MAPSIZE(x)                  (((x) / (BYTES_PER_DWORD) + 1) * sizeof(DWORD))
#define POS2SHIFTCOUNT(x)       (DWORD)  (HIGHEST_NIBBLE_BIT - (((x) & NIBBLES_PER_DWORD_MASK) << LOG2_NIBBLE_SIZE))
#define POS2MASK(x)             (DWORD) ~(HIGHEST_NIBBLE_MASK >> (((x) & NIBBLES_PER_DWORD_MASK) << LOG2_NIBBLE_SIZE))

namespace NibbleMap {

    inline DWORD Pos2ShiftCount(size_t pos)
    {
        return HIGHEST_NIBBLE_BIT - ((pos & NIBBLES_PER_DWORD_MASK) << LOG2_NIBBLE_SIZE);
    }
    
    inline size_t GetDwordIndex(size_t relativePointer)
    {
        return relativePointer >> LOG2_BYTES_PER_DWORD;
    }

    inline size_t GetNibbleIndex(size_t relativePointer)
    {
        return (relativePointer >> (LOG2_BYTES_PER_BUCKET)) & NIBBLES_PER_DWORD_MASK;
    }

    inline size_t NibbleToRelativeAddress(size_t dwordIndex, size_t nibbleIndex, DWORD nibbleValue)
    {
        return (dwordIndex << LOG2_BYTES_PER_DWORD) + (nibbleIndex << LOG2_BYTES_PER_BUCKET) + ((nibbleValue - 1) << LOG2_CODE_ALIGN);
    }

    inline DWORD GetNibble(DWORD dword, size_t nibbleIndex)
    {
        return (dword >> POS2SHIFTCOUNT(nibbleIndex)) & NIBBLE_MASK;
    }

    namespace LinearLookupNibbleMap
    {
        inline void SetUnlocked(PTR_DWORD pMap, TADDR mapBase, TADDR pCode, BOOL bSet)
        {
            CONTRACTL {
                NOTHROW;
                GC_NOTRIGGER;
            } CONTRACTL_END;

            _ASSERTE(pCode >= mapBase);

            size_t delta = pCode - mapBase;

            size_t pos  = ADDR2POS(delta);
            DWORD value = bSet?ADDR2OFFS(delta):0;

            DWORD index = (DWORD) (pos >> LOG2_NIBBLES_PER_DWORD);
            DWORD mask  = ~((DWORD) HIGHEST_NIBBLE_MASK >> ((pos & NIBBLES_PER_DWORD_MASK) << LOG2_NIBBLE_SIZE));

            value = value << POS2SHIFTCOUNT(pos);

            // assert that we don't overwrite an existing offset
            // (it's a reset or it is empty)
            _ASSERTE(!value || !((*(pMap+index))& ~mask));

            // It is important for this update to be atomic. Synchronization would be required with FindMethodCode otherwise.
            *(pMap+index) = ((*(pMap+index))&mask)|value;
        }

        inline TADDR FindMethodCode(PTR_DWORD pMap, TADDR mapBase, TADDR startAddress, TADDR endAddress, PCODE currentPC)
        {
            LIMITED_METHOD_DAC_CONTRACT;

            if ((currentPC < startAddress) ||
                (currentPC > endAddress))
            {
                return 0;
            }

            TADDR base = mapBase;
            TADDR delta = currentPC - base;
            PTR_DWORD pMapStart = pMap;

            DWORD tmp;

            size_t startPos = ADDR2POS(delta);  // align to 32byte buckets
                                                // ( == index into the array of nibbles)
            DWORD  offset   = ADDR2OFFS(delta); // this is the offset inside the bucket + 1

            _ASSERTE(offset == (offset & NIBBLE_MASK));

            pMap += (startPos >> LOG2_NIBBLES_PER_DWORD); // points to the proper DWORD of the map

            // get DWORD and shift down our nibble

            PREFIX_ASSUME(pMap != NULL);
            tmp = VolatileLoadWithoutBarrier<DWORD>(pMap) >> POS2SHIFTCOUNT(startPos);

            if ((tmp & NIBBLE_MASK) && ((tmp & NIBBLE_MASK) <= offset) )
            {
                return base + POSOFF2ADDR(startPos, tmp & NIBBLE_MASK);
            }

            // Is there a header in the remainder of the DWORD ?
            // go to an earlier nibble
            tmp = tmp >> NIBBLE_SIZE;

            // if that is initialized at all find it and return that pointer
            if (tmp)
            {
                startPos--;
                while (!(tmp & NIBBLE_MASK))
                {
                    tmp = tmp >> NIBBLE_SIZE;
                    startPos--;
                }
                return base + POSOFF2ADDR(startPos, tmp & NIBBLE_MASK);
            }

            // We skipped the remainder of the DWORD,
            // so we must set startPos to the highest position of
            // previous DWORD, unless we are already on the first DWORD

            if (startPos < NIBBLES_PER_DWORD)
                return 0;

            startPos = ((startPos >> LOG2_NIBBLES_PER_DWORD) << LOG2_NIBBLES_PER_DWORD) - 1;

            // Skip "headerless" DWORDS

            while (pMapStart < pMap && 0 == (tmp = VolatileLoadWithoutBarrier<DWORD>(--pMap)))
            {
                startPos -= NIBBLES_PER_DWORD;
            }

            // This helps to catch degenerate error cases. This relies on the fact that
            // startPos cannot ever be bigger than MAX_UINT
            if (((INT_PTR)startPos) < 0)
                return 0;

            // Find the nibble with the header in the DWORD

            while (startPos && !(tmp & NIBBLE_MASK))
            {
                tmp = tmp >> NIBBLE_SIZE;
                startPos--;
            }

            if (startPos == 0 && tmp == 0)
                return 0;

            return base + POSOFF2ADDR(startPos, tmp & NIBBLE_MASK);
        }
    }

    namespace ConstantLookupNibbleMap
    {
        inline bool IsPointer(DWORD dword)
        {
            return GetNibble(dword, NIBBLES_PER_DWORD - 1) > 8;
        }

        inline DWORD EncodePointer(size_t relativePointer)
        {
            return (DWORD) ((relativePointer & ~NIBBLE_MASK) + (((relativePointer & NIBBLE_MASK) >> 2) + 9));
        }

        inline size_t DecodePointer(DWORD dword)
        {
            return (size_t) ((dword & ~NIBBLE_MASK) + (((dword & NIBBLE_MASK) - 9) << 2));
        }

        // Write Algo:
        //     1. Write nibble as in previous algorithm
        //     2. If function completely covers following DWORDs, insert relative pointers
        inline void SetUnlocked(PTR_DWORD pMap, TADDR mapBase, TADDR pCode, size_t codeSize){
            CONTRACTL {
                NOTHROW;
                GC_NOTRIGGER;
            } CONTRACTL_END;

            _ASSERTE(pCode >= mapBase);

            size_t delta = pCode - mapBase;

            size_t dwordIndex = GetDwordIndex(delta);
            size_t nibbleIndex = GetNibbleIndex(delta);
            DWORD value = ADDR2OFFS(delta);

            DWORD mask  = POS2MASK(nibbleIndex);

            value = value << Pos2ShiftCount(nibbleIndex);

            // assert that we don't overwrite an existing offset
            // the nibble is empty and the DWORD is not a pointer
            _ASSERTE(!((*(pMap+dwordIndex)) & ~mask) && !IsPointer(*(pMap+dwordIndex)));

            // It is important for this update to be atomic. Synchronization would be required with FindMethodCode otherwise.
            *(pMap+dwordIndex) = ((*(pMap+dwordIndex)) & mask) | value;

            size_t firstByteAfterMethod = delta + codeSize;
            size_t nextDwordIndex = dwordIndex + 1;
            while ((nextDwordIndex + 1) * BYTES_PER_DWORD <= firstByteAfterMethod)
            {
                // All of these DWORDs should be empty
                _ASSERTE(!(*(pMap+nextDwordIndex)));
                *(pMap+nextDwordIndex) = EncodePointer(delta);
                nextDwordIndex++;
            }
        }

        inline void DeleteUnlocked(PTR_DWORD pMap, TADDR mapBase, TADDR pCode)
        {
            CONTRACTL {
                NOTHROW;
                GC_NOTRIGGER;
            } CONTRACTL_END;

            _ASSERTE(pCode >= mapBase);

            size_t delta = pCode - mapBase;

            size_t dwordIndex = GetDwordIndex(delta);
            size_t nibbleIndex = GetNibbleIndex(delta);

            DWORD mask  = POS2MASK(nibbleIndex);

            // assert that the nibble is not empty and the DWORD is not a pointer
            _ASSERTE(((*(pMap+dwordIndex)) & ~mask) && !IsPointer(*(pMap+dwordIndex)));

            // delete the relevant nibble
            // It is important for this update to be atomic. Synchronization would be required with FindMethodCode otherwise.
            *(pMap+dwordIndex) = ((*(pMap+dwordIndex)) & mask);

            // the last DWORD of the nibble map is reserved to be empty for bounds checking
            while (IsPointer(*(pMap+dwordIndex+1))){
                // The next DWORD is a pointer, so we can delete it
                *(pMap+dwordIndex+1) = 0;
                dwordIndex++;
            }
        }

        // Read Algo:
        //     1. Look up DWORD representing given PC
        //     2. If DWORD is a Pointer, then we are done. Otherwise,
        //     3. If nibble corresponding to PC is initialized, if the value the nibble represents precedes the PC return that value, otherwise
        //     4. Find the next preceding initialized nibble in the DWORD. If found, return the value the nibble represents. Otherwise,
        //     5. Execute steps 1, 2 then 4 on the proceeding DWORD. If this is also uninitialized, then we are not in a function and can return a nullptr.
        inline TADDR FindMethodCode(PTR_DWORD pMap, TADDR mapBase, TADDR startAddress, TADDR endAddress, PCODE currentPC)
        {
            LIMITED_METHOD_DAC_CONTRACT;

            if ((currentPC < startAddress) ||
                (currentPC > endAddress))
            {
                return 0;
            }

            TADDR base = mapBase;
            TADDR delta = currentPC - base;

            size_t dwordIndex = GetDwordIndex(delta);
            size_t nibbleIndex = GetNibbleIndex(delta);

            DWORD dword;
            DWORD nibble;

            size_t startPos = ADDR2POS(delta);  // align to 32byte buckets
                                                // ( == index into the array of nibbles)
            DWORD  offset   = ADDR2OFFS(delta); // this is the offset inside the bucket + 1
            _ASSERTE(offset == (offset & NIBBLE_MASK));

            // #1 look up DWORD represnting current PC
            PREFIX_ASSUME(pMap != NULL);
            dword = VolatileLoadWithoutBarrier<DWORD>(pMap + dwordIndex);

            // #2 if DWORD is a pointer, then we can return
            if (IsPointer(dword))
            {
                return base + DecodePointer(dword);
            }

            // #3 if DWORD is nibbles and corresponding nibble is intialized and points to an equal or earlier address, return the corresponding address
            nibble = GetNibble(dword, nibbleIndex);
            if ((nibble) && (nibble <= offset) )
            {
                return base + NibbleToRelativeAddress(dwordIndex, nibbleIndex, nibble);
            }

            // #4 find preceeding nibble and return if found
            // TODO: re-implement using ctz intrinsic
            for(; nibbleIndex-- > 0;)
            {
                nibble = GetNibble(dword, nibbleIndex);
                if (nibble)
                {
                    return base + NibbleToRelativeAddress(dwordIndex, nibbleIndex, nibble);
                }
            }

            // #5.1 read previous DWORD. If no such DWORD return 0.
            if (dwordIndex == 0)
            {
                return 0;
            }
            dwordIndex--;
            nibbleIndex = NIBBLES_PER_DWORD - 1;

            PREFIX_ASSUME(pMap != NULL);
            dword = VolatileLoadWithoutBarrier<DWORD>(pMap + dwordIndex);

            // #5.2 if DWORD is a pointer, then we can return
            if (IsPointer(dword))
            {
                return base + DecodePointer(dword);
            }

            // #5.4 find preceeding nibble and return if found
            for(; nibbleIndex-- > 0;)
            {
                nibble = GetNibble(dword, nibbleIndex);
                if (nibble)
                {
                    return base + NibbleToRelativeAddress(dwordIndex, nibbleIndex, nibble);
                }
            }

            // If none of the above was found, return 0
            return 0;
        }
    }

    inline void SetUnlocked(PTR_DWORD pMap, TADDR mapBase, TADDR pCode, size_t codeSize)
    {
        CONTRACTL {
            NOTHROW;
            GC_NOTRIGGER;
        } CONTRACTL_END;

        if (CLRConfig::GetConfigValue(CLRConfig::INTERNAL_UseOptimizedNibbleMap))
        {
            return ConstantLookupNibbleMap::SetUnlocked(pMap, mapBase, pCode, codeSize);
        }
        else
        {
            return LinearLookupNibbleMap::SetUnlocked(pMap, mapBase, pCode, true);
        }
    }

    inline void DeleteUnlocked(PTR_DWORD pMap, TADDR mapBase, TADDR pCode)
    {
        CONTRACTL {
            NOTHROW;
            GC_NOTRIGGER;
        } CONTRACTL_END;

        if (CLRConfig::GetConfigValue(CLRConfig::INTERNAL_UseOptimizedNibbleMap))
        {
            return ConstantLookupNibbleMap::DeleteUnlocked(pMap, mapBase, pCode);
        }
        else
        {
            return LinearLookupNibbleMap::SetUnlocked(pMap, mapBase, pCode, false);
        }
    }

    inline TADDR FindMethodCode(PTR_DWORD pMap, TADDR mapBase, TADDR startAddress, TADDR endAddress, PCODE currentPC)
    {
        CONTRACTL {
            NOTHROW;
            GC_NOTRIGGER;
        } CONTRACTL_END;

        if (CLRConfig::GetConfigValue(CLRConfig::INTERNAL_UseOptimizedNibbleMap))
        {
            return ConstantLookupNibbleMap::FindMethodCode(pMap, mapBase, startAddress, endAddress, currentPC);
        }
        else
        {
            return LinearLookupNibbleMap::FindMethodCode(pMap, mapBase, startAddress, endAddress, currentPC);
        }
    }
}

#endif  // NIBBLEMAPMACROS_H_
