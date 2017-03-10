//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_ITERATOR_H
#define EFFICIENTGRAPH_LIB_ITERATOR_H

namespace base{

    template <typename G, typename _Item>
    class ItemIterator {};

    template <typename G>
    class ItemIterator<G, typename G::Node>{
    public:
        typedef G Graph;
        typedef typename G::Node Item;
        typedef typename G::NodeIt ItemIt;

        template <typename V>
        class Map : public G::template NodeMap<V>{
            typedef typename G :: template NodeMap<V> Parent;
        public:
            typedef typename G::template NodeMap<V> Type;
            typedef typename Parent::Value Value;

            Map(const G & _graph) : Parent (_graph) {}
            Map(const G & _graph, const Value & _value) : Parent (_graph, _value) {}
        };
    };

    template <typename G>
    class ItemIterator<G, typename G::Arc>{
    public:
        typedef G Graph;
        typedef typename G::Arc Item;
        typedef typename G::ArcIt ItemIt;

        template <typename V>
        class Map : public G::template ArcMap<V>{
            typedef typename G :: template ArcMap<V> Parent;
        public:
            typedef typename G::template ArcMap<V> Type;
            typedef typename Parent::Value Value;

            Map(const G & _graph) : Parent (_graph) {}
            Map(const G & _graph, const Value & _value) : Parent (_graph, _value) {}
        };
    };
}

#endif //EFFICIENTGRAPH_LIB_ITERATOR_H
