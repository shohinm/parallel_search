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

#ifndef SMPL_INTRUSIVE_HEAP_HPP
#define SMPL_INTRUSIVE_HEAP_HPP

#include "intrusive_heap.h"

#include <assert.h>
#include <stdio.h>

namespace smpl {

template <class T, class Compare>
intrusive_heap<T, Compare>::intrusive_heap(const compare& comp) :
    m_data(1, nullptr),
    m_comp(comp)
{
}

template <class T, class Compare>
template <class InputIt>
intrusive_heap<T, Compare>::intrusive_heap(
    const compare& comp,
    InputIt first,
    InputIt last)
:
    m_data(),
    m_comp(comp)
{
    make_heap(first, last);
}

template <class T, class Compare>
template <class InputIt>
intrusive_heap<T, Compare>::intrusive_heap(InputIt first, InputIt last) :
    m_data(),
    m_comp()
{
    make_heap(first, last);
}

template <class T, class Compare>
intrusive_heap<T, Compare>::intrusive_heap(intrusive_heap&& o) :
    m_data(std::move(o.m_data)),
    m_comp(std::move(o.m_comp))
{
}

template <class T, class Compare>
intrusive_heap<T, Compare>&
intrusive_heap<T, Compare>::operator=(intrusive_heap&& rhs)
{
    if (this != &rhs) {
        m_data = std::move(rhs.m_data);
        m_comp = std::move(rhs.m_comp);
    }
    return *this;
}

template <class T, class Compare>
T* intrusive_heap<T, Compare>::min() const
{
    assert(m_data.size() > 1);
    return m_data[1];
}

template <class T, class Compare>
typename intrusive_heap<T, Compare>::const_iterator
intrusive_heap<T, Compare>::begin() const
{
    return m_data.begin() + 1;
}

template <class T, class Compare>
typename intrusive_heap<T, Compare>::const_iterator
intrusive_heap<T, Compare>::end() const
{
    return m_data.end();
}

template <class T, class Compare>
bool intrusive_heap<T, Compare>::empty() const
{
    return m_data.size() == 1;
}

template <class T, class Compare>
typename intrusive_heap<T, Compare>::size_type
intrusive_heap<T, Compare>::size() const
{
    return m_data.size() - 1;
}

template <class T, class Compare>
typename intrusive_heap<T, Compare>::size_type
intrusive_heap<T, Compare>::max_size() const
{
    return m_data.max_size() - 1;
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::reserve(size_type new_cap)
{
    m_data.reserve(new_cap + 1);
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::clear()
{
    for (size_t i = 1; i < m_data.size(); ++i) {
        m_data[i]->m_heap_index = 0;
    }
    m_data.resize(1);
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::push(T* e)
{
    assert(e);
    e->m_heap_index = m_data.size();
    m_data.push_back(e);
    percolate_up(m_data.size() - 1);
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::pop()
{
    assert(!empty());
    m_data[1]->m_heap_index = 0;
    m_data[1] = m_data.back();
    // NOTE: no need to set heap index here, as it is not required in
    // percolate_down(); also produces incorrect behavior when popping from a
    // heap of size 1 without the required identity check here
//    m_data[1]->m_heap_index = 1;
    m_data.pop_back();
    percolate_down(1);
}

template <class T, class Compare>
bool intrusive_heap<T, Compare>::contains(T* e)
{
    assert(e);
    return e->m_heap_index != 0;
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::update(T* e)
{
    assert(e && contains(e));
    percolate_up(e->m_heap_index);
    percolate_down(e->m_heap_index);
}

template <typename T, class Compare>
void intrusive_heap<T, Compare>::increase(T* e)
{
    assert(e && contains(e));
    percolate_down(e->m_heap_index);
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::decrease(T* e)
{
    assert(e && contains(e));
    percolate_up(e->m_heap_index);
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::erase(T* e)
{
    assert(e && contains(e));
    size_type pos = e->m_heap_index;
    m_data[pos] = m_data.back();
    m_data[pos]->m_heap_index = pos;
    e->m_heap_index = 0;
    m_data.pop_back();
    if ((pos < m_data.size()) && contains(m_data[pos]))
    {
        update(m_data[pos]);
    }
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::make()
{
    for (auto i = (m_data.size() - 1) >> 1; i >= 1; --i) {
        percolate_down(i);
    }
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::swap(intrusive_heap& o)
{
    if (this != &o) {
        using std::swap;
        swap(m_data, o.m_data);
        swap(m_comp, o.m_comp);
    }
}

template <class T, class Compare>
inline
typename intrusive_heap<T, Compare>::size_type
intrusive_heap<T, Compare>::ipow2(size_type exp)
{
    if (exp == 0) {
        return 1;
    }

    size_type res = ipow2(exp >> 1) * ipow2(exp >> 1);
    if (exp % 2) {
        res *= 2;
    }
    return res;
}

template <class T, class Compare>
inline
typename intrusive_heap<T, Compare>::size_type
intrusive_heap<T, Compare>::ilog2(size_type i)
{
    std::size_t r = 0;
    while (i >>= 1) {
        ++r;
    }
    return r;
}

template <class T, class Compare>
inline
bool intrusive_heap<T, Compare>::ispow2(size_type val)
{
    // does not check for val == 0
    return !(val & (val - 1));
}

template <class T, class Compare>
template <class InputIt>
void intrusive_heap<T, Compare>::make_heap(InputIt first, InputIt last)
{
    auto n = std::distance(first, last);

    m_data.clear();
    m_data.reserve(n + 1);
    m_data.push_back(nullptr);
    m_data.insert(m_data.end(), first, last);

    for (size_type i = 1; i < m_data.size(); ++i) {
        m_data[i]->m_heap_index = i;
    }

    for (auto i = n >> 1; i >= 1; --i) {
        percolate_down(i);
    }
}

template <class T, class Compare>
template <class InputIt>
void intrusive_heap<T, Compare>::make_heap(
    InputIt first,
    InputIt last,
    size_type root)
{
    const auto n = std::distance(first, last);
    printf("make heap from %d elements from %zu\n", n, root);
    print();

    if (n <= 0) {
        return;
    }

    printf(" -> data[%zu] = %p\n", root, *first);
    m_data[root] = *first;
    m_data[root]->m_heap_index = root;

    if (n == 1) {
        return;
    }

    const size_type left = left_child(root);
    const size_type right = right_child(root);

    auto f = ilog2(n) - 1;
    size_type f2 = ipow2(f);
    size_type l = f2 - 1 + std::min(n - 2 * f2 + 1, f2);
    size_type r = n - 1 - l;

    InputIt new_start = std::next(first);
    InputIt mid = std::next(new_start, l);

    make_heap(new_start, mid, left);
    make_heap(mid, last, right);
    percolate_down(root);
}

template <class T, class Compare>
inline
typename intrusive_heap<T, Compare>::size_type
intrusive_heap<T, Compare>::parent(size_type index) const
{
    return index >> 1;
}

template <class T, class Compare>
inline
typename intrusive_heap<T, Compare>::size_type
intrusive_heap<T, Compare>::right_child(size_type index) const
{
    return (index << 1) + 1;
}

template <class T, class Compare>
inline
typename intrusive_heap<T, Compare>::size_type
intrusive_heap<T, Compare>::left_child(size_type index) const
{
    return index << 1;
}

template <class T, class Compare>
inline
void intrusive_heap<T, Compare>::percolate_down(size_type pivot)
{
    if (is_external(pivot)) {
        return;
    }

    size_type left = left_child(pivot);
    size_type right = right_child(pivot);

    T* tmp = m_data[pivot];
    while (is_internal(left)) {
        size_type s = right;
        if (is_external(right) || m_comp(*m_data[left], *m_data[right])) {
            s = left;
        }

        if (m_comp(*m_data[s], *tmp)) {
            m_data[pivot] = m_data[s];
            m_data[pivot]->m_heap_index = pivot;
            pivot = s;
        } else {
            break;
        }

        left = left_child(pivot);
        right = right_child(pivot);
    }
    m_data[pivot] = tmp;
    m_data[pivot]->m_heap_index = pivot;
}

template <class T, class Compare>
inline
void intrusive_heap<T, Compare>::percolate_up(size_type pivot)
{
    T* tmp = m_data[pivot];
    while (pivot != 1) {
        size_type p = parent(pivot);
        if (m_comp(*m_data[p], *tmp)) {
            break;
        }
        m_data[pivot] = m_data[p];
        m_data[pivot]->m_heap_index = pivot;
        pivot = p;
    }
    m_data[pivot] = tmp;
    m_data[pivot]->m_heap_index = pivot;
}

template <class T, class Compare>
inline
bool intrusive_heap<T, Compare>::is_internal(size_type index) const
{
    return index < m_data.size();
}

template <class T, class Compare>
inline
bool intrusive_heap<T, Compare>::is_external(size_type index) const
{
    return index >= m_data.size();
}

template <class T, class Compare>
void intrusive_heap<T, Compare>::print() const
{
    printf("[ null, ");
    for (int i = 1; i < m_data.size(); ++i) {
        printf(" (%d, %p)", m_data[i]->m_heap_index, m_data[i]);
        if (i == m_data.size() - 1) {
            printf(" ");
        } else {
            printf(", ");
        }
    }
    printf("]\n");
}

template <class T, class Compare>
bool intrusive_heap<T, Compare>::check_heap(size_type index) const
{
    auto right = right_child(index);
    if (is_internal(right)) {
        // check the ordering of the parent and the right child
        if (!m_comp(*m_data[index], *m_data[right])) {
            return false;
        }

        // check the tree rooted at the right child
        if (!check_heap(right)) {
            return false;
        }
    }

    auto left = left_child(index);
    if (is_internal(left)) {
        // check the ordering of the parent and the left child
        if (!m_comp(*m_data[index], *m_data[left])) {
            return false;
        }

        // check the tree rooted at the left child
        if (!check_heap(left)) {
            return false;
        }
    }

    return true;
}

template <class T, class Compare>
bool intrusive_heap<T, Compare>::check_heap() const
{
    if (empty()) {
        return true;
    }

    return check_heap(1);
}

template <class T, class Compare>
void swap(intrusive_heap<T, Compare>& lhs, intrusive_heap<T, Compare>& rhs)
{
    lhs.swap(rhs);
}

} // namespace smpl

#endif
