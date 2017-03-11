//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_ITERATOR_H
#define EFFICIENTGRAPH_LIB_ITERATOR_H

#include "lib_base.h"

namespace base{
    //--------------------------------------------iterator wrapper----------------------------------------------------------
    template <class T>
    struct IteratorWrapper : public T , public std::iterator<std::input_iterator_tag, T> {
        IteratorWrapper(const T &x) : T(x) {}
        const T &operator*() const { return static_cast<const T &>(*this); }
        const T *operator->(){ return static_cast<const T *>(this); }
        void operator++(int){ T::operator++; }
        using T::operator++;
    };

    template <class LIT, class P>
    class Iterator1{
        typedef IteratorWrapper<LIT> It;
        It _begin;
    public:
        Iterator1(const P &p) : _begin(LIT(p)) {}
        It begin() const { return _begin; }
        It end() const { return It(INVALID); }
    };

    template <class LIT, class P1, class P2>
    class Iterator2{
        typedef IteratorWrapper<LIT> It;
        It _begin;
    public:
        Iterator2(const P1 &p1, const P2 &p2) : _begin(LIT(p1, p2)) {}
        It begin() const { return _begin; }
        It end() const { return It(INVALID); }
    };
}

#endif //EFFICIENTGRAPH_LIB_ITERATOR_H
