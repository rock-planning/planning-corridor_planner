#ifndef NAV_MEMORY_POOL_HH
#define NAV_MEMORY_POOL_HH

#include <list>
#include <vector>
#include <boost/cstdint.hpp>

namespace corridor_planner
{
    template<typename _Tp, typename _Alloc = std::allocator<_Tp> >
    class pool_allocator
    {
    public:
        typedef size_t     size_type;
        typedef ptrdiff_t  difference_type;
        typedef _Tp*       pointer;
        typedef const _Tp* const_pointer;
        typedef _Tp&       reference;
        typedef const _Tp& const_reference;
        typedef _Tp        value_type;

        size_type __size;
        std::list< std::vector<boost::uint8_t> > __pools;
        void* __freelist;
        _Alloc __alloc;
        static const int POOL_SIZE = 1000;
        static const int ALIGN     = 4;

        template<typename _Tp1>
            struct rebind
            { typedef pool_allocator<_Tp1> other; };

        static size_type compute_size()
        {
            size_type result = sizeof(_Tp);
            if (result < sizeof(void*))
                result = sizeof(void*);

            return ((result + ALIGN - 1) / ALIGN) * ALIGN;
        }
        pool_allocator() throw() : __size(compute_size()), __freelist(0) { }

        pool_allocator(const pool_allocator&) throw() : __size(compute_size()), __freelist(0) { }

        template<typename _Tp1>
            pool_allocator(const pool_allocator<_Tp1>&) throw() : __size(compute_size()), __freelist(0) { }

        ~pool_allocator() throw() { }

        pointer
            address(reference __x) const { return &__x; }

        const_pointer
            address(const_reference __x) const { return &__x; }

        // NB: __n is permitted to be 0.  The C++ standard says nothing
        // about what the return value is when __n == 0.
        pointer
            allocate(size_type __n)
            { 
                if (__n > __size)
                {
                    return __alloc.allocate(__n);
                }

                if (!__freelist)
                {
                    // Allocate one more buffer and fill the freelist with it
                    __pools.push_back( std::vector<uint8_t>() );
                    std::vector<uint8_t>& new_pool = __pools.back();
                    new_pool.resize(__size * POOL_SIZE);
                    for (int i = 0; i < POOL_SIZE; ++i)
                    {
                        void* slot = &new_pool[__size * i];
                        *static_cast<void**>(slot) = __freelist;
                        __freelist = slot;
                    }
                }

                // Remove the front element from the freelist and return
                // it
                void* slot = __freelist;
                __freelist = *static_cast<void**>(__freelist);
                return static_cast<_Tp*>(slot);
            }

        // __p is not permitted to be a null pointer.
        void
            deallocate(pointer __p, size_type __n)
            {
                if (__n > __size)
                {
                    __alloc.deallocate(__p, __n);
                }
                else
                {
                    void* slot = __p;
                    *static_cast<void**>(slot) = __freelist;
                    __freelist = slot;
                }
            }

        size_type
            max_size() const throw() 
            { return size_t(-1) / sizeof(_Tp); }

        // _GLIBCXX_RESOLVE_LIB_DEFECTS
        // 402. wrong new expression in [some_] allocator::construct
        void 
            construct(pointer __p, const _Tp& __val) 
            { ::new((void *)__p) _Tp(__val); }

        void 
            destroy(pointer __p) { __p->~_Tp(); }
    };

    template<typename _Tp>
        inline bool
        operator==(const pool_allocator<_Tp>&, const pool_allocator<_Tp>&)
        { return true; }

    template<typename _Tp>
        inline bool
        operator!=(const pool_allocator<_Tp>&, const pool_allocator<_Tp>&)
        { return false; }

}
#endif

