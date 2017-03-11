//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_GRAPH_H
#define EFFICIENTGRAPH_LIB_GRAPH_H

#include "lib_base.h"
#include "lib_map.h"
#include "lib_iterator.h"
#include <iostream>

namespace base{
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

        //-----------------collection of nodes----------
        Iterator1<NodeIt, Graph> nodes() const{
            return Iterator1<NodeIt, Graph> (*this);
        };

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

        //------------------collection of out arcs------
        Iterator2<OutArcIt, Graph, Node> outArcs(const Node & u) const{
            return Iterator2<OutArcIt, Graph, Node>(*this, u);
        };

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

        //------------------collection of in arcs------
        Iterator2<InArcIt, Graph, Node> inArcs(const Node & u) const{
            return Iterator2<InArcIt, Graph, Node>(*this, u);
        };

        //------------------iterator go each-----------
        class ArcIt : public Arc{
        public:
            ArcIt () {}
            ArcIt (const ArcIt &e) : Arc(e) {}
            ArcIt (Invalid) {}
            explicit ArcIt(const Graph &) {}
            ArcIt(const Graph &, const Arc &) {}
            ArcIt &operator++() { return *this; }
        };

        Iterator1<ArcIt, Graph> arcs() const{
            return Iterator1<ArcIt, Graph> (*this);
        };

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

    //-----------------------------------------list graph base define---------------------------------------------------
    class ListGraphBase{
    protected:

        typedef struct NodeT{
            int first_in, first_out;
            int prev, next;
        }NodeType;

        typedef struct ArcT{
            int target, source;
            int prev_in, prev_out;
            int next_in, next_out;
        }ArcType;

        typedef std::vector<NodeType> NodeVector;
        typedef std::vector<ArcType> ArcVector;

        int first_node;
        int first_free_node;

        NodeVector _nodes;
        ArcVector _arcs;

        int first_free_arc;

    public:
        class Node{
            friend class ListGraph;
            friend class ListGraphBase;

        protected:

            int id;
            explicit Node(int pid) { id = pid; }

        public:

            Node() {}
            Node (Invalid) { id = -1; }
            bool operator==(const Node &node) const { return id == node.id; }
            bool operator!=(const Node &node) const { return id != node.id; }
            bool operator< (const Node &node) const { return id < node.id; }
            bool operator> (const Node &node) const { return id < node.id; }
        };

        class Arc{
            friend class ListGraph;
            friend class ListGraphBase;

        protected:

            int id;
            explicit Arc(int pid) { id = pid; }

        public:

            Arc() {}
            Arc (Invalid) { id = -1; }
            bool operator==(const Arc &arc) const { return id == arc.id; }
            bool operator!=(const Arc &arc) const { return id != arc.id; }
            bool operator< (const Arc &arc) const { return id < arc.id; }
            bool operator> (const Arc &arc) const { return id < arc.id; }
        };

        ListGraphBase() : _nodes(), first_node(-1), first_free_node(-1), _arcs(-1), first_free_arc(-1) {}
        int maxNodeId() const { return _nodes.size() - 1; }
        int maxArcId() const { return _arcs.size() - 1; }

        Node source(Arc e) const { return Node(_arcs[e.id].source); }
        Node target(Arc e) const { return Node(_arcs[e.id].source); }

        void first(Node & node) const { node.id = first_node; }
        void next(Node & node) const { node.id = _nodes[node.id].next; }

        void first(Arc &arc) const {
            int n;
            for(n = first_node; n!=-1 && _nodes[n].first_out == -1; n = _nodes[n].next) {}
            arc.id = (n == -1) ? -1 : _nodes[n].first_out;
        }

        void next(Arc &arc) const {
            if(_arcs[arc.id].next_out != -1){
                arc.id = _arcs[arc.id].next_out;
            } else {
                int n;
                for(n = _nodes[_arcs[arc.id].source].next; n!=-1 && _nodes[n].first_out == -1; n = _nodes[n].next) {}
                arc.id = (n == -1) ? -1 : _nodes[n].first_out;
            }
        }

        void firstOut(Arc &a, const Node & n) const {
            a.id = _nodes[n.id].first_out;
        }
        void nextOut(Arc &a) const {
            a.id = _arcs[a.id].next_out;
        }
        void firstIn(Arc &a, const Node &n) const{
            a.id = _nodes[n.id].first_in;
        }
        void nextIn(Arc &a) const {
            a.id = _arcs[a.id].next_in;
        }
        static int id(Node n) { return n.id; }
        static int id(Arc a) { return a.id; }

        static Node nodeFromId(int id) { return Node(id); }
        static Arc arcFromId(int id) { return Arc(id); }

    public:
        bool valid(Node n) const{
            return n.id >= 0 && n.id < static_cast<int>(_nodes.size()) && _nodes[n.id].prev != -2;
        }
        bool valid(Arc a) const {
            return a.id >=0 && a.id < static_cast<int>(_arcs.size()) && _nodes[a.id].prev != -2;
        }

        Node addNode(){
            int n;
            if(first_free_node == -1){
                n = _nodes.size();
                _nodes.push_back(NodeT());
            } else {
                n = first_free_node;
                first_free_node = _nodes[n].next;
            }
            _nodes[n].next = first_node;
            if(first_node != -1) _nodes[first_node].prev = n;
            first_node = n;
            _nodes[n].prev = -1;
            _nodes[n].first_in = _nodes[n].first_out - 1;
            return Node(n);
        }

        Arc addArc(Node u, Node v){
            int n;
            if(first_free_arc == -1){
                n = _arcs.size();
                _arcs.push_back(ArcT());
            } else {
                n = first_free_arc;
                first_free_arc = _arcs[n].next_in;
            }

            _arcs[n].source = u.id;
            _arcs[n].target = v.id;

            _arcs[n].next_out = _nodes[u.id].first_out;
            if(_nodes[u.id].first_out != -1){
                _arcs[_nodes[u.id].first_out].prev_out = n;
            }

            _arcs[n].next_in = _nodes[u.id].first_in;
            if(_nodes[v.id].first_in != -1){
                _arcs[_nodes[v.id].first_in].prev_in = n;
            }

            _arcs[n].prev_in = _arcs[n].prev_out = -1;
            _nodes[u.id].first_out = _nodes[v.id].first_in = n;
            return Arc(n);
        }

        void erase(const Node &node){
            int n = node.id;
            if(_nodes[n].next != -1){
                _nodes[_nodes[n].next].prev = _nodes[n].prev;
            }

            if(_nodes[n].prev != -1){
                _nodes[_nodes[n].prev].next = _nodes[n].next;
            } else {
                first_node = _nodes[n].next;
            }

            _nodes[n].next = first_free_node;
            first_free_node = n;
            _nodes[n].prev = -2;
        }

        void erase(const Arc &arc){
            int n = arc.id;

            if(_arcs[n].next_in != -1){
                _arcs[_arcs[n].next_in].prev_in = _arcs[n].prev_in;
            }

            if(_arcs[n].prev_in != -1){
                _arcs[_arcs[n].prev_in].next_in = _arcs[n].next_in;
            } else {
                _nodes[_arcs[n].target].first_in = _arcs[n].next_in;
            }

            if(_arcs[n].next_out != -1){
                _arcs[_arcs[n].next_out].prev_out = _arcs[n].prev_out;
            }

            if(_arcs[n].prev_out != -1){
                _arcs[_arcs[n].prev_out].next_out = _arcs[n].next_out;
            } else {
                _nodes[_arcs[n].source].first_out = _arcs[n].next_out;
            }

            _arcs[n].next_in = first_free_arc;
            first_free_arc = n;
            _arcs[n].prev_in = -2;
        }

        void clear(){
            _arcs.clear();
            _nodes.clear();
            first_node = first_free_node = first_free_arc = -1;
        }

    protected:

        void changeTarget(Arc e, Node n)
        {
            if(_arcs[e.id].next_in != -1)
                _arcs[_arcs[e.id].next_in].prev_in = _arcs[e.id].prev_in;
            if(_arcs[e.id].prev_in != -1)
                _arcs[_arcs[e.id].prev_in].next_in = _arcs[e.id].next_in;
            else _nodes[_arcs[e.id].target].first_in = _arcs[e.id].next_in;
            if (_nodes[n.id].first_in != -1) {
                _arcs[_nodes[n.id].first_in].prev_in = e.id;
            }
            _arcs[e.id].target = n.id;
            _arcs[e.id].prev_in = -1;
            _arcs[e.id].next_in = _nodes[n.id].first_in;
            _nodes[n.id].first_in = e.id;
        }
        void changeSource(Arc e, Node n)
        {
            if(_arcs[e.id].next_out != -1)
                _arcs[_arcs[e.id].next_out].prev_out = _arcs[e.id].prev_out;
            if(_arcs[e.id].prev_out != -1)
                _arcs[_arcs[e.id].prev_out].next_out = _arcs[e.id].next_out;
            else _nodes[_arcs[e.id].source].first_out = _arcs[e.id].next_out;
            if (_nodes[n.id].first_out != -1) {
                _arcs[_nodes[n.id].first_out].prev_out = e.id;
            }
            _arcs[e.id].source = n.id;
            _arcs[e.id].prev_out = -1;
            _arcs[e.id].next_out = _nodes[n.id].first_out;
            _nodes[n.id].first_out = e.id;
        }
    };
    //-------------------------------------end list graph base define---------------------------------------------------

    //------------------------------------------------graph extender define---------------------------------------------
    template <typename Base>
    class GraphExtender : public Base{
        typedef Base Parent;

    public:

        typedef GraphExtender Graph;
        typedef typename Parent::Node Node;
        typedef typename Parent::Arc Arc;

        int maxId(Node) const { return Parent::maxNodeId(); }
        int maxId(Arc) const { return Parent::maxArcId(); }
        static Node fromId(int id, Node) { return Parent::nodeFromId(id); }
        static Arc fromId(int id, Arc) { return Parent::arcFromId(id); }

        class NodeIt : public Node {
            const Graph* _graph;
        public:

            NodeIt() {}

            NodeIt(Invalid i) : Node(i) { }

            explicit NodeIt(const Graph& graph) : _graph(&graph) {
                _graph->first(static_cast<Node&>(*this));
            }

            NodeIt(const Graph& graph, const Node& node)
                    : Node(node), _graph(&graph) {}

            NodeIt& operator++() {
                _graph->next(*this);
                return *this;
            }

        };

        class ArcIt : public Arc {
            const Graph* _graph;
        public:

            ArcIt() { }

            ArcIt(Invalid i) : Arc(i) { }

            explicit ArcIt(const Graph& graph) : _graph(&graph) {
                _graph->first(static_cast<Arc&>(*this));
            }

            ArcIt(const Graph& graph, const Arc& arc) :
                    Arc(arc), _graph(&graph) { }

            ArcIt& operator++() {
                _graph->next(*this);
                return *this;
            }

        };

        class OutArcIt : public Arc {
            const Graph* _graph;
        public:

            OutArcIt() { }

            OutArcIt(Invalid i) : Arc(i) { }

            OutArcIt(const Graph& graph, const Node& node)
                    : _graph(&graph) {
                _graph->firstOut(*this, node);
            }

            OutArcIt(const Graph& graph, const Arc& arc)
                    : Arc(arc), _graph(&graph) {}

            OutArcIt& operator++() {
                _graph->nextOut(*this);
                return *this;
            }

        };

        class InArcIt : public Arc {
            const Graph* _graph;
        public:

            InArcIt() { }

            InArcIt(Invalid i) : Arc(i) { }

            InArcIt(const Graph& graph, const Node& node)
                    : _graph(&graph) {
                _graph->firstIn(*this, node);
            }

            InArcIt(const Graph& graph, const Arc& arc) :
                    Arc(arc), _graph(&graph) {}

            InArcIt& operator++() {
                _graph->nextIn(*this);
                return *this;
            }

        };

        Node addNode(){
            Node node = Parent::addNode();
            return node;
        }

        Arc addArc(const Node &from, const Node &to){
            Arc arc = Parent::addArc(from, to);
            return arc;
        }

        void clear(){ Parent::clear(); }

        template <typename Graph, typename NodeRefMap, typename ArcRefMap>
        void build(const Graph & graph, NodeRefMap &nodeRef, ArcRefMap &arcRef){
            Parent::build(graph, nodeRef, arcRef);
        };

        void erase(const Node &node){
            Arc arc;
            Parent::firstOut(arc, node);
            while(arc != INVALID){
                erase(arc);
                Parent::firstOut(arc, node);
            }
            Parent::firstIn(arc, node);
            while(arc != INVALID){
                erase(arc);
                Parent::firstIn(arc, node);
            }
            Parent::erase(node);
        }

        void erase(const Arc & arc){ Parent::erase(arc); }
    };
    //--------------------------------------------end graph extender define---------------------------------------------

    //------------------------------------------------list graph define-------------------------------------------------
    typedef GraphExtender<ListGraphBase> Extended;
    class ListGraph : public Extended{
        typedef Extended Parent;
    private:
        ListGraph(const ListGraph &) : Extended() {};
        void operator=(const ListGraph &) {}

    public:

        ListGraph() {}
        Node addNode() { return Parent::addNode(); }
        Arc addArc(Node s, Node d) { return Parent::addArc(s,d);}
        void erase(Node n) { Parent::erase(n); }
        void erase(Arc a) { Parent::erase(a); }
        void clear() { Parent::clear(); }
        bool valid(Node n) const { Parent::valid(n); }
        bool valid(Arc a) const { Parent::valid(a); }
        void changeTarget(Arc a, Node n){ Parent::changeTarget(a, n); }
        void changeSource(Arc a, Node n){ Parent::changeSource(a,n); }
        void reverseArc(Arc a){
            Node t = target(a);
            changeTarget(a,source(a));
            changeSource(a,t);
        }
        void contract(Node u, Node v, bool r = true){
            for(OutArcIt e(*this,v); e != INVALID;){
                OutArcIt f = e;
                ++f;
                if(r && target(e) == u) erase(e);
                else changeSource(e,u);
                e = f;
            }
            for(InArcIt e(*this,v); e != INVALID;){
                InArcIt f = e;
                ++f;
                if(r && source(e) == u) erase(e);
                else changeSource(e,u);
                e = f;
            }
            erase(v);
        }
        Node split(Node n, bool connect = true){
            Node b = addNode();
            _nodes[b.id].first_out = _nodes[n.id].first_out;
            _nodes[n.id].first_out = -1;
            for(int i=_nodes[b.id].first_out; i != -1; i=_arcs[i].next_out){
                _arcs[i].source = b.id;
            }
            if(connect) addArc(n,b);
            return b;
        }
        Node split(Arc a){
            Node v = addNode();
            addArc(v,target(a));
            changeTarget(a,v);
            return v;
        }
        void reserveNode(int n) { _nodes.reserve(n); }
        void reserveArc(int m) { _arcs.reserve(m); }

    };
    //--------------------------------------------end list graph define-------------------------------------------------
}

#endif //EFFICIENTGRAPH_GRAPH_LIB_H
