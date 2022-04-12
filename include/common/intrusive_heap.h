////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

#ifndef SMPL_INTRUSIVE_HEAP_H
#define SMPL_INTRUSIVE_HEAP_H

#include <cstdlib>
#include <vector>

namespace smpl {

template <class T, class Compare>
class intrusive_heap;

struct heap_element
{

    heap_element() : m_heap_index(0) { }

private:

    std::size_t m_heap_index;

    template <class T, class Compare>
    friend class intrusive_heap;
};

/// Provides an intrusive binary heap implementation. Objects inserted into the
/// heap must derive from the \p heap_element class to implement efficient
/// mutability. The implementation stores pointers to inserted objects, which
/// must remain valid throughout the lifetime of the heap.
///
/// The binary heap data structure provides constant time access to the minimum
/// element of the heap, logarithmic insertion and erasure of elements,
/// logarithmic updates of element priorities, and linear time construction from
/// a new set of elements. All times are proportional to the number of elements
/// in the heap.
///
/// Priorities of elements in the heap are not explicitly stored, but determined
/// from the result of calling the \p Compare function object on two elements.
/// If the priorities of multiple elements are implicitly changed (via external
/// modification of the function object), the heap may be reordered in-place
/// in linear time by calling the make() member function.
template <class T, class Compare>
class intrusive_heap
{
public:

    static_assert(std::is_base_of<heap_element, T>::value, "T must extend heap_element");

    typedef Compare compare;

    typedef std::vector<T*> container_type;
    typedef typename container_type::size_type size_type;

    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;

    intrusive_heap(const compare& comp = compare());

    template <class InputIt>
    intrusive_heap(InputIt first, InputIt last);

    template <class InputIt>
    intrusive_heap(const compare& comp, InputIt first, InputIt last);

    intrusive_heap(const intrusive_heap&) = delete;

    intrusive_heap(intrusive_heap&& o);

    intrusive_heap& operator=(const intrusive_heap&) = delete;
    intrusive_heap& operator=(intrusive_heap&& rhs);

    T* min() const;

    const_iterator begin() const;
    const_iterator end() const;

    bool empty() const;
    size_type size() const;
    size_type max_size() const;
    void reserve(size_type new_cap);

    void clear();
    void push(T* e);
    void pop();
    bool contains(T* e);
    void update(T* e);
    void increase(T* e);
    void decrease(T* e);
    void erase(T* e);

    void make();

    void swap(intrusive_heap& o);

private:

    container_type m_data;
    Compare m_comp;

    size_type ipow2(size_type i);
    size_type ilog2(size_type i);
    bool ispow2(size_type val);

    template <class InputIt>
    void make_heap(InputIt first, InputIt last);

    template <class InputIt>
    void make_heap(InputIt first, InputIt last, size_type root);

    size_type parent(size_type index) const;
    size_type right_child(size_type index) const;
    size_type left_child(size_type index) const;

    void percolate_down(size_type pivot);
    void percolate_up(size_type pivot);

    bool is_internal(size_type index) const;
    bool is_external(size_type index) const;

    void print() const;

    bool check_heap() const;
    bool check_heap(size_type index) const;
};

template <class T, class Compare>
void swap(intrusive_heap<T, Compare>& lhs, intrusive_heap<T, Compare>& rhs);

} // namespace smpl

#include "intrusive_heap.hpp"

#endif
