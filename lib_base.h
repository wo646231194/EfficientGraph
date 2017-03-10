//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_BASE_H
#define EFFICIENTGRAPH_LIB_BASE_H

#include "lib_map.h"
#include "lib_graph.h"
#include "lib_iterator.h"

namespace base{

    typedef Graph::Node Node;
    typedef Graph::NodeIt NodeIt;
    typedef Graph::Arc Arc;
    typedef Graph::ArcIt ArcIt;
    typedef Graph::InArcIt InArcIt;
    typedef Graph::OutArcIt OutArcIt;
    typedef Graph::template NodeMap<int> IntNodeMap;
    typedef Graph::template ArcMap<int> IntArcMap;

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
