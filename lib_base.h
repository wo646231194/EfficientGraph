//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_BASE_H
#define EFFICIENTGRAPH_LIB_BASE_H

namespace base{
    //------------------------------------------------iterator----------------------------------------------------------
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
    //----------------------------------------------------INVALID-------------------------------------------------------
    struct Invalid{
    public:
        bool operator==(Invalid) { return true; }
        bool operator!=(Invalid) { return false; }
        bool operator< (Invalid) { return false; }
        bool operator> (Invalid) { return false; }
    };
    const Invalid INVALID = Invalid();

#define Graph_Typedef(Graph)                           \
    typedef typename Graph::Node Node;                          \
    typedef typename Graph::NodeIt NodeIt;                      \
    typedef typename Graph::Arc Arc;                            \
    typedef typename Graph::ArcIt ArcIt;                        \
    typedef typename Graph::InArcIt InArcIt;                    \
    typedef typename Graph::OutArcIt OutArcIt;                  \
    typedef typename Graph::template NodeMap<int> IntNodeMap;   \
    typedef typename Graph::template ArcMap<int> IntArcMap;

    template <typename Graph, typename Item>
    inline int countItems(const Graph & g){
        typedef typename ItemIterator<Graph, Item>::ItemIt ItemIt;
        int num = 0;
        for(ItemIt it(g); it !=INVALID; ++it){
            ++num;
        }
        return num;
    };

    template <typename Graph>
    inline int countNode(const Graph & g){
        return countItems<Graph, typename Graph::Node>(g);
    }

    template <typename Graph>
    inline int countArc(const Graph & g){
        return countItems<Graph, typename Graph::Arc>(g);
    }
}

#endif //EFFICIENTGRAPH_LIB_BASE_H
