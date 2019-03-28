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

#include <engine/memory.h>

namespace sfge
{


LinearAllocator::LinearAllocator(size_t size, void* start) : Allocator(size, start), _current_pos(start) { assert(size > 0); }

LinearAllocator::~LinearAllocator() { _current_pos = nullptr; }

void* LinearAllocator::allocate(size_t size, ptr_type alignment)
{
    assert(size != 0);
    ptr_type adjustment = alignForwardAdjustment(_current_pos, alignment);

    if(_used_memory + adjustment + size > _size)
        return nullptr;

    ptr_type aligned_address = (ptr_type)_current_pos + adjustment;
    _current_pos = (void*)(aligned_address + size);
    _used_memory += size + adjustment;
    _num_allocations++;

    return (void*)aligned_address;
}

void LinearAllocator::deallocate(void* p)
{
    assert( false && "Use clear() instead" );
}

void LinearAllocator::clear()
{
    _num_allocations = 0;
    _used_memory = 0;
    _current_pos = _start;
}


StackAllocator::StackAllocator(size_t size, void* start) : Allocator(size, start), _current_pos(start)
{
    assert(size > 0);
#if _DEBUG
    _prev_position = nullptr;
#endif
}

StackAllocator::~StackAllocator()
{
#if _DEBUG
    _prev_position = nullptr;
#endif

    _current_pos = nullptr;
}

void* StackAllocator::allocate(size_t size, ptr_type alignment)
{
    assert(size != 0);
    ptr_type adjustment = alignForwardAdjustmentWithHeader(_current_pos, alignment, sizeof(AllocationHeader));

    if(_used_memory + adjustment + size > _size) return nullptr;

    void* aligned_address = (void*)((ptr_type) _current_pos + adjustment);

    //Add Allocation Header 
    auto* header = (AllocationHeader*)((ptr_type)aligned_address- sizeof(AllocationHeader));
    header->adjustment = adjustment;

#if _DEBUG
    header->prev_address = _prev_position;
    _prev_position = aligned_address;
#endif

    _current_pos = (void*)((ptr_type)aligned_address + size);
    _used_memory += size + adjustment;
    _num_allocations++;

    return aligned_address;
}

void StackAllocator::deallocate(void* p)
{
#if _DEBUG
    assert( p == _prev_position );
#endif
    //Access the AllocationHeader in the bytes before p 
    auto* header = (AllocationHeader*)(((ptr_type)p - sizeof(AllocationHeader)));
    _used_memory -= (ptr_type)_current_pos - (ptr_type)p + header->adjustment;
    _current_pos = (void*)((ptr_type)p - header->adjustment);

#if _DEBUG
    _prev_position = header->prev_address;
#endif

    _num_allocations--;
}

FreeListAllocator::FreeListAllocator(size_t size, void* start) : Allocator(size, start), _free_blocks((FreeBlock*)start)
{
    assert(size > sizeof(FreeBlock));
    _free_blocks->size = size;
    _free_blocks->next = nullptr;
}

FreeListAllocator::~FreeListAllocator()
{
    _free_blocks = nullptr;
}

void* FreeListAllocator::allocate(size_t size, ptr_type alignment)
{
    assert(size != 0 && alignment != 0);
    FreeBlock* prev_free_block = nullptr;
    FreeBlock* free_block = _free_blocks;

    while(free_block != nullptr)
    {
        //Calculate adjustment needed to keep object correctly aligned 
        ptr_type adjustment = alignForwardAdjustmentWithHeader(free_block, alignment, sizeof(AllocationHeader));
        size_t total_size = size + adjustment;

        //If allocation doesn't fit in this FreeBlock, try the next 
        if(free_block->size < total_size)
        {
            prev_free_block = free_block;
            free_block = free_block->next;
            continue;
        }

        static_assert(sizeof(AllocationHeader) >= sizeof(FreeBlock), "sizeof(AllocationHeader) < sizeof(FreeBlock)");

        //If allocations in the remaining memory will be impossible 
        if(free_block->size - total_size <= sizeof(AllocationHeader))
        {
            //Increase allocation size instead of creating a new FreeBlock 
            total_size = free_block->size;

            if(prev_free_block != nullptr)
                prev_free_block->next = free_block->next;
            else
                _free_blocks = free_block->next;
        }
        else
        {
            //Else create a new FreeBlock containing remaining memory 
            auto* next_block = (FreeBlock*)( ((ptr_type) free_block+total_size) );

            next_block->size = free_block->size - total_size;
            next_block->next = free_block->next;

            if(prev_free_block != nullptr)
                prev_free_block->next = next_block;
            else
                _free_blocks = next_block;
        }

        ptr_type aligned_address = (ptr_type)free_block + adjustment;
        AllocationHeader* header = (AllocationHeader*)(aligned_address - sizeof(AllocationHeader));
        header->size = total_size;
        header->adjustment = adjustment;
        _used_memory += total_size;
        _num_allocations++;

        assert(alignForwardAdjustment((void*)aligned_address, alignment) == 0);

        return (void*)aligned_address;
    }

    //assert(false && "Couldn't find free block large enough!"); 
    return nullptr;
}

void FreeListAllocator::deallocate(void* p)
{
    assert(p != nullptr);
    AllocationHeader* header = (AllocationHeader*)((ptr_type)p - sizeof(AllocationHeader));
    ptr_type block_start = reinterpret_cast<ptr_type>(p) - header->adjustment;
    size_t block_size = header->size;
    ptr_type block_end = block_start + block_size;
    FreeBlock* prev_free_block = nullptr;
    FreeBlock* free_block = _free_blocks;

    while(free_block != nullptr)
    {
        if( (ptr_type) free_block >= block_end ) break;
        prev_free_block = free_block;
        free_block = free_block->next;
    }

    if(prev_free_block == nullptr)
    {
        prev_free_block = (FreeBlock*) block_start;
        prev_free_block->size = block_size;
        prev_free_block->next = _free_blocks;
        _free_blocks = prev_free_block;
    }
    else if((ptr_type) prev_free_block + prev_free_block->size == block_start)
    {
        prev_free_block->size += block_size;
    }
    else
    {
        FreeBlock* temp = (FreeBlock*) block_start;
        temp->size = block_size;
        temp->next = prev_free_block->next;
        prev_free_block->next = temp;
        prev_free_block = temp;
    }

    if( free_block != nullptr && (ptr_type) free_block == block_end)
    {
        prev_free_block->size += free_block->size;
        prev_free_block->next = free_block->next;
    }

    _num_allocations--;
    _used_memory -= block_size;
}

PoolAllocator::PoolAllocator(size_t objectSize, ptr_type objectAlignment, size_t size, void* mem) : Allocator(size, mem), _objectSize(objectSize), _objectAlignment(objectAlignment)
{
    assert(objectSize >= sizeof(void*));

    //Calculate adjustment needed to keep object correctly aligned 
    ptr_type adjustment = alignForwardAdjustment(mem, objectAlignment);
    _free_list = (void**)((ptr_type)mem + adjustment);
    size_t numObjects = (size-adjustment)/objectSize;
    void** p = _free_list;

    //Initialize free blocks list 
    for(size_t i = 0; i < numObjects-1; i++)
    {
        *p = (p + objectSize );
        p = (void**) *p;
    }

    *p = nullptr;
}

PoolAllocator::~PoolAllocator()
{
    _free_list = nullptr;
}

void* PoolAllocator::allocate(size_t size, ptr_type alignment)
{
    assert(size == _objectSize && alignment == _objectAlignment);
    if(_free_list == nullptr) return nullptr;
    void* p = _free_list;
    _free_list = (void**)(*_free_list);
    _used_memory += size;
    _num_allocations++;
    return p;
}

void PoolAllocator::deallocate(void* p)
{
    *((void**)p) = _free_list;
    _free_list = (void**)p;
    _used_memory -= _objectSize;
    _num_allocations--;
}

ProxyAllocator::ProxyAllocator(Allocator& allocator) : Allocator(allocator.getSize(), allocator.getStart()), _allocator(allocator) { }

ProxyAllocator::~ProxyAllocator() { }

void* ProxyAllocator::allocate(size_t size, ptr_type alignment)
{
    assert(size != 0);
    _num_allocations++;
    size_t mem = _allocator.getUsedMemory();

    void* p = _allocator.allocate(size, alignment);
    _used_memory += _allocator.getUsedMemory() - mem;
    return p;
}

void ProxyAllocator::deallocate(void* p)
{
    _num_allocations--;
    size_t mem = _allocator.getUsedMemory();
    _allocator.deallocate(p);
    _used_memory -= mem - _allocator.getUsedMemory();
}

}