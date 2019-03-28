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

#include <gtest/gtest.h>
#include <iostream>

#include <engine/memory.h>

TEST(Memory, TestCustomAllocator)
{
    void* data = calloc(1024*1024, sizeof(char));
    sfge::LinearAllocator linearAllocator(1032, data); //Counting header size
    int array_size = 1024/sizeof(int);

    int* integer_data = sfge::allocator::allocateArray<int>(linearAllocator, array_size);
    for(int i = 0; i<array_size;i++)
    {
        integer_data[i] = rand();
    }
    int sum = 0;
    for(int i = 0; i < array_size;i++)
    {
        sum += integer_data[i];
    }
    std::cout << "Number sum: " << sum << " with memory used: " << linearAllocator.getUsedMemory() << " and memory allocations: "<< linearAllocator.getNumAllocations() << "\n";
    linearAllocator.clear();
    free(data);

}

