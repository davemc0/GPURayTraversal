
#pragma once
#include "base/Defs.hpp"

// An allocator that uses an STL vector as its backing store and provides individual elements of the array when requested
// Cannot reallocate because it would invalidate the pointers to elements
template<class T>
class ArrayAllocator
{
    struct AllocUnion
    {
        size_t nextEmpty; // Index of next empty element; 0 => ALL higher elements are empty
    };

public:

    ArrayAllocator() : m_items(nullptr), m_size(0), m_firstEmpty(0), m_count(0) {}
    
    void init(T* dataPtr, size_t N)
    {
        FW_ASSERT(sizeof(T) >= sizeof(AllocUnion));
        FW_ASSERT(m_size == 0);
        FW_ASSERT(m_count == 0);

        // m_items = (T*)malloc(N * sizeof(T));
        m_items = dataPtr;
        m_size = N;
        m_firstEmpty = 0;
        m_count = 0;

        T* ptr = m_items;
        AllocUnion* A = (AllocUnion*)(void*)ptr;
        A->nextEmpty = 0;
    }

    void deinit()
    {
        // Deinit DOES NOT invalidate all the pointers to nodes.
        // free(m_items); // XXX
        m_size = 0;
        m_count = 0;
        m_firstEmpty = 0;
    }

    // Allocates one T on the array
    T* alloc(size_t )
    {
        //FW_ASSERT(bytes == sizeof(T));
       // printf("0x%016llx alloc() %lld %lld\n", (__int64)m_items, m_firstEmpty, m_count);
        if (m_firstEmpty >= m_size) {
            FW_ASSERT(!!!"ItemArrayAllocator is full");
            return nullptr;
        }

        T* ptr = &m_items[m_firstEmpty];

        AllocUnion* A = (AllocUnion*)(void*)ptr;
        m_firstEmpty = A->nextEmpty ? A->nextEmpty : (m_firstEmpty + 1);
        if (A->nextEmpty == 0) {
            AllocUnion* B = (AllocUnion*)(void*)&m_items[m_firstEmpty];
            B->nextEmpty = 0;
        }
        m_count++;
        FW_ASSERT(m_count <= m_size);

        return ptr;
    }

    void free(T* ptr)
    {
        size_t ind = ptr - &m_items[0];
        FW_ASSERT(ind < m_size);
        //printf("0x%016llx free(%lld) %lld %lld\n", (__int64)m_items, ind, m_firstEmpty, m_count);

        AllocUnion* A = (AllocUnion*)(void*)ptr;
        A->nextEmpty = m_firstEmpty;
        m_firstEmpty = ind;
        m_count--;

        if (m_count == 0) {
            // Garbage collect if whole array is empty
            m_firstEmpty = 0;
            AllocUnion* F = (AllocUnion*)m_items;
            F->nextEmpty = 0;
        }
    }

    T* data() { return m_items; }
    const T* data() const { return m_items; }

    T& begin() { return m_items[0]; }
    T& end()   { return *(m_items + m_size); }

private:
    T*     m_items; // This class does not own the data. Thus, this class can be deleted and the generated data will survive.
    size_t m_size; // Allocated size
    size_t m_firstEmpty; // Index of first empty one in the linked list
    size_t m_count; // Number currently inserted
};
