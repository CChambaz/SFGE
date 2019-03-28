/*
 MIT License

 Copyright (c) 2017 SAE Institute Switzerland AG

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */


#ifndef SFGE_MEMORY_H
#define SFGE_MEMORY_H

#include <cstdlib>
#include <cstdint>
#include <cassert>

#include <engine/globals.h>

#ifdef IS64BIT
using ptr_type = long;
#endif

#ifdef IS32BIT
using ptr_type = int;
#endif


namespace sfge
{
class Allocator
{

public:

    Allocator(size_t size, void *start)
    {
        _start = start;
        _size = size;
        _used_memory = 0;
        _num_allocations = 0;
    }

    virtual ~Allocator()
    {
        assert(_num_allocations == 0 && _used_memory == 0);
        _start = nullptr;
        _size = 0;
    }

    virtual void *allocate(size_t size, ptr_type alignment = 4) = 0;

    virtual void deallocate(void *p) = 0;

    void *getStart() const { return _start; }

    size_t getSize() const { return _size; }

    size_t getUsedMemory() const { return _used_memory; }

    size_t getNumAllocations() const { return _num_allocations; }

protected:

    void *_start;
    size_t _size;
    size_t _used_memory;
    size_t _num_allocations;
};

namespace allocator
{
    template<class T>
    T *allocateNew(Allocator &allocator)
    {
        return new(allocator.allocate(sizeof(T), __alignof(T))) T;
    }

    template<class T>
    T *allocateNew(Allocator &allocator, const T &t)
    {
        return new(allocator.allocate(sizeof(T), __alignof(T))) T(t);
    }

    template<class T>
    void deallocateDelete(Allocator &allocator, T &object)
    {
        object.~T();
        allocator.deallocate(&object);
    }

    template<class T>
    T *allocateArray(Allocator &allocator, size_t length)
    {
        assert(length != 0);
        int headerSize = sizeof(size_t) / sizeof(T);

        if (sizeof(size_t) % sizeof(T) > 0) headerSize += 1;

        //Allocate extra space to store array length in the bytes before the array
        T *p = ((T *) allocator.allocate(sizeof(T) * (length + headerSize), __alignof(T))) + headerSize;

        *(((size_t *) p) - 1) = length;

        for (size_t i = 0; i < length; i++)
            new(&p) T;

        return p;
    }

    template<class T>
    void deallocateArray(Allocator &allocator, T *array)
    {
        assert(array != nullptr);
        size_t length = *(((size_t *) array) - 1);

        for (size_t i = 0; i < length; i++) array->~T();

        //Calculate how much extra memory was allocated to store the length before the array
        int headerSize = sizeof(size_t) / sizeof(T);
        if (sizeof(size_t) % sizeof(T) > 0)
            headerSize += 1;
        allocator.deallocate(array - headerSize);
    }
};

inline void* alignForward(void* address, ptr_type alignment)
{
    return (void*)( (((ptr_type)static_cast<int*>(address)) + (alignment-1) ) & (~(alignment-1)) );
}

inline ptr_type alignForwardAdjustment(const void* address, ptr_type alignment)
{
    ptr_type adjustment = alignment - ((ptr_type)static_cast<const int*>(address) & (alignment-1) );

    if(adjustment == alignment) return 0;

    //already aligned
    return adjustment;
}

inline ptr_type alignForwardAdjustmentWithHeader(const void* address, ptr_type alignment, ptr_type headerSize)
{
    ptr_type adjustment = alignForwardAdjustment(address, alignment);
    ptr_type neededSpace = headerSize;

    if(adjustment < neededSpace)
    {
        neededSpace -= adjustment;

        //Increase adjustment to fit header
        adjustment += alignment * (neededSpace / alignment);

        if(neededSpace % alignment > 0) adjustment += alignment;
    }

    return adjustment;
}

class LinearAllocator : public Allocator
{
public:

    LinearAllocator(size_t size, void* start);
    ~LinearAllocator();

    void* allocate(size_t size, ptr_type alignment) override;
    void deallocate(void* p) override;
    void clear();

private:

    LinearAllocator(const LinearAllocator&);

    //Prevent copies because it might cause errors
    LinearAllocator& operator=(const LinearAllocator&);
    void* _current_pos;
};

class StackAllocator : public Allocator
{
public:

    StackAllocator(size_t size, void* start);
    ~StackAllocator();

    void* allocate(size_t size, ptr_type alignment) override;
    void deallocate(void* p) override;

private:

    StackAllocator(const StackAllocator&);

    //Prevent copies because it might cause errors
    StackAllocator& operator=(const StackAllocator&);

    struct AllocationHeader
    {
#if _DEBUG
        void* prev_address;
#endif

        ptr_type adjustment;
    };

#if _DEBUG
    void* _prev_position;
#endif

    void* _current_pos;
};


class FreeListAllocator : public Allocator
{
public:

    FreeListAllocator(size_t size, void* start);
    ~FreeListAllocator();

    void* allocate(size_t size, ptr_type alignment) override;
    void deallocate(void* p) override;

private:

    struct AllocationHeader { size_t size; ptr_type adjustment; };
    struct FreeBlock { size_t size; FreeBlock* next; };
    FreeListAllocator(const FreeListAllocator&);

    //Prevent copies because it might cause errors
    FreeListAllocator& operator=(const FreeListAllocator&);
    FreeBlock* _free_blocks;
};

class PoolAllocator : public Allocator
{
public:

    PoolAllocator(size_t objectSize, ptr_type objectAlignment, size_t size, void* mem);
    ~PoolAllocator();
    void* allocate(size_t size, ptr_type alignment) override;
    void deallocate(void* p) override;

private:

    PoolAllocator(const PoolAllocator&);

    //Prevent copies because it might cause errors 
    PoolAllocator& operator=(const PoolAllocator&); size_t _objectSize;

    ptr_type _objectAlignment;
    void** _free_list;
};

class ProxyAllocator : public Allocator
{
public:

    ProxyAllocator(Allocator& allocator);
    ~ProxyAllocator();
    void* allocate(size_t size, ptr_type alignment) override;
    void deallocate(void* p) override;

private:

    ProxyAllocator(const ProxyAllocator&);

    //Prevent copies because it might cause errors
    ProxyAllocator& operator=(const ProxyAllocator&);
    Allocator& _allocator;
};

}
#endif