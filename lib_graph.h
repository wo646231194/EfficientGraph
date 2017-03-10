//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_GRAPH_H
#define EFFICIENTGRAPH_LIB_GRAPH_H

#include "lib_map.h"

namespace base{
    //----------------------------------------------------INVALID-------------------------------------------------------
    struct Invalid{
    public:
        bool operator==(Invalid) { return true; }
        bool operator!=(Invalid) { return false; }
        bool operator< (Invalid) { return false; }
        bool operator> (Invalid) { return false; }
    };
    const Invalid INVALID = Invalid();

    //---------------------------------------------------graph define---------------------------------------------------
    class Graph{
    private:
        //---------------constructor-------------
        Graph(const Graph &) {}
        //---------------operator---------------
        void operator=(const Graph &) {}

    public:
        Graph() {}

        //------------------node define-----------------
        class Node{
        public:
            Node() {}
            Node(const Node &) {}
            Node(Invalid) {}

            bool operator==(Node) const { return true; }
            bool operator!=(Node) const { return true; }
            bool operator< (Node) const { return true; }
            bool operator> (Node) const { return true; }
        };

        //------------------node iterator define--------
        class NodeIt : public Node{
        public:
            NodeIt () {}
            NodeIt(const NodeIt &n) : Node(n) {}
            NodeIt(Invalid) {}
            explicit NodeIt(const Graph &) {}
            NodeIt(const Graph &, const Node &) {}
            NodeIt &operator++() { return *this; }
        };

//        //-----------------collection of nodes----------
//        Wrapper1<NodeIt, Graph> node() const{
//            return Wrapper1<NodeIt, Graph> (*this);
//        };

        //-----------------arc define-------------------
        class Arc{
        public:
            Arc () {}
            Arc (const Arc &) {}
            Arc (Invalid) {}
            bool operator==(Arc) const { return true; }
            bool operator!=(Arc) const { return true; }
            bool operator< (Arc) const { return true; }
            bool operator> (Arc) const { return true; }
        };

        //---------------arc out define-----------------
        class OutArcIt : public Arc{
        public:
            OutArcIt () {}
            OutArcIt (const OutArcIt &e) : Arc(e) {}
            OutArcIt (Invalid) {}
            OutArcIt (const Graph &, const Node &) {}
            OutArcIt (const Graph &, const Arc &) {}
            OutArcIt &operator++() { return *this; }
        };

//        //------------------collection of out arcs------
//        Wrapper2<OurArcIt, Graph, Node> ourArc(const Node & u) const{
//            return Wrapper2<OurArcIt, Graph, Node>(*this, u);
//        };

        //---------------arc in define-----------------
        class InArcIt : public Arc{
        public:
            InArcIt () {}
            InArcIt (const InArcIt &e) : Arc(e) {}
            InArcIt (Invalid) {}
            InArcIt (const Graph &, const Node &) {}
            InArcIt (const Graph &, const Arc &) {}
            InArcIt &operator++() { return *this; }
        };

//        //------------------collection of in arcs------
//        Wrapper2<InArcIt, Graph, Node> ourArc(const Node & u) const{
//            return Wrapper2<InArcIt, Graph, Node>(*this, u);
//        };

        //------------------iterator go each-----------
        class ArcIt : public Arc{
            ArcIt () {}
            ArcIt (const ArcIt &e) : Arc(e) {}
            ArcIt (Invalid) {}
            explicit ArcIt(const Graph &) {}
            ArcIt(const Graph &, const Arc &) {}
            ArcIt &operator++() { return *this; }
        };

//        Wrapper1<ArcIt, Graph> node() const{
//            return Wrapper1<ArcIt, Graph> (*this);
//        };

        Node source(Arc) const { return INVALID; }
        Node target(Arc) const { return INVALID; }
        int id(Node) const { return -1; }
        int id(Arc) const { return -1; }
        Node nodeFromId(int) const { return INVALID; }
        Node arcFromId(int) const { return INVALID; }
        int maxNodeId() const { return -1; }
        int maxArcId() const { return -1; }

        void first(Node &) const {}
        void next(Node &) const {}
        void first(Arc &) const {}
        void next(Arc &) const {}

        void firstIn(Arc &, const Node &) const {}
        void nextIn(Arc &) const {}
        void firstOut(Arc &, const Node &) const {}
        void nextOut(Arc &) const {}

        Node fromId(int, Node) const { return INVALID; }
        Arc fromId(int, Arc) const {return INVALID; }
        int maxId(Node) const { return -1; }
        int maxId(Arc) const { return -1; }

        Node baseNode(OutArcIt) const { return INVALID; }
        Node runningNode(OutArcIt) const { return INVALID; }
        Node baseNode(InArcIt) const { return INVALID; }
        Node runningNode(InArcIt) const { return INVALID; }

        template<typename V>
        class NodeMap : public ReferenceMap<Node, V, V &, const V &>{
        public:
            explicit NodeMap(const Graph &) {}
            NodeMap(const Graph &, V) {}

        private:
            NodeMap(const NodeMap &nm) : ReferenceMap<Node, V, V &, const V &> (nm) {}
            template <typename CMap>
            NodeMap &operator=(const CMap &){ return *this; }
        };

        template<typename V>
        class ArcMap : public ReferenceMap<Arc, V, V &, const V &>{
        public:
            explicit ArcMap(const Graph &) {}
            ArcMap(const Graph &, V) {}

        private:
            ArcMap(const ArcMap &nm) : ReferenceMap<Arc, V, V &, const V &> (nm) {}
            template <typename CMap>
            ArcMap &operator=(const CMap &){ return *this; }
        };
    };
    //----------------------------------------------end  graph define---------------------------------------------------
}

#endif //EFFICIENTGRAPH_GRAPH_LIB_H
