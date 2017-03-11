//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_SOLVER_H
#define EFFICIENTGRAPH_LIB_SOLVER_H

#include "lib_base.h"
#include <limits>
#include <algorithm>

namespace base{
    //--------------------------------------------------simplex solver--------------------------------------------------
    namespace simplex{
        template<typename G>
        class solver{
        private:
            Graph_Typedef(G);

            typedef std::vector<int> IntVector;
            typedef std::vector<signed char> CharVector;

            //------------arc state---------
            enum ArcState{
                STATE_UPPER = -1,
                STATE_TREE = 0,
                STATE_LOWER = 1
            };
            //------------arc direction-----
            enum ArcDirection{
                DIRECTION_DOWN=-1,
                DIRECTION_UP = 1
            };

            enum ProblemType {
                INFEASIBLE,
                OPTIMAL,
                UNBOUNDED
            };

            enum PivotRule {
                FIRST_ELIGIBLE
            };

            const G & _graph;
            int _node_num; // node number
            int _arc_num; // arc number
            int _all_arc_num;
            int _search_arc_num;

            bool _has_lower;
            int _sum_supply; // sum all node's supply
            int _root;

            IntNodeMap _node_id;
            IntArcMap _arc_id;
            IntVector _source;
            IntVector _target;
            bool _arc_mixing;

            IntVector _lower; //low of arc
            IntVector _upper; //up of arc
            IntVector _cap; //cap of arc
            IntVector _cost; //cost of arc
            IntVector _cost_pi;
            IntVector _flow; //flow of arc
            IntVector _supply; // supply of node

            IntVector _parent;
            IntVector _pred;
            IntVector _thread;
            IntVector _rev_thread;
            IntVector _succ_num;
            IntVector _last_succ;
            CharVector _pred_dir;
            CharVector _state;
            IntVector _dirty_revs;

            int in_arc, join, u_in, v_in, u_out, v_out;
            int delta;

            const int MAX = std::numeric_limits<int>::max();

        public:

            const int INF = MAX;

        private:

            //----------------------------------------------search method-----------------------------------------
            class FirstSearch{
            private:
                const IntVector &_source;
                const IntVector &_target;
                const IntVector &_cost;
                const CharVector &_state;
                const IntVector &_cost_pi;
                int &_in_arc;
                int _search_arc_num;
                int _next_arc;

            public:
                //---------------------------constructor-------------------------
                FirstSearch(solver &sss) :
                        _source(sss._source), _target(sss._target),
                        _cost(sss._cost), _state(sss._state), _cost_pi(sss._cost_pi),
                        _in_arc(sss.in_arc), _search_arc_num(sss._search_arc_num),_next_arc(0) {}

                bool findEnteringArc(){
                    int c;
                    for(int e = _next_arc; e!= _search_arc_num; ++e){
                        c = _state[e] * (_cost[e] + _cost_pi[_source[e]] - _cost_pi[_target[e]]);
                        if(c<0){
                            _in_arc = e;
                            _next_arc = e + 1;
                            return true;
                        }
                    }
                    return false;
                }
            };
            //------------------------------------------search method end-----------------------------------------

        public:
            //-----------------------------------------solver constructor-----------------------------------------
            solver(const G & graph, bool arc_mixing = true) :
                    _graph(graph), _node_id(graph), _arc_id(graph), _arc_mixing(arc_mixing){
                reset();
            }

            solver & resetParams(){
                for(int i=0; i!=_node_num; ++i){
                    _supply[i] = 0;
                }
                for(int i=0; i!=_arc_num; ++i){
                    _lower[i] = 0;
                    _upper[i] = INF;
                    _cost[i] = 0;
                }
                _has_lower = false;
                return *this;
            }

            solver & reset(){
                _node_num = countNode(_graph);
                _arc_num = countArc(_graph);
                int all_node_num = _node_num + 1;
                int max_arc_num = _arc_num + 2 * _node_num;

                _source.resize(max_arc_num);
                _target.resize(max_arc_num);

                _lower.resize(_arc_num);
                _upper.resize(_arc_num);
                _cap.resize(max_arc_num);
                _cost.resize(max_arc_num);
                _supply.resize(all_node_num);
                _flow.resize(max_arc_num);
                _cost_pi.resize(all_node_num);

                _parent.resize(all_node_num);
                _pred.resize(all_node_num);
                _pred_dir.resize(all_node_num);
                _thread.resize(all_node_num);
                _rev_thread.resize(all_node_num);
                _succ_num.resize(all_node_num);
                _last_succ.resize(all_node_num);
                _state.resize(max_arc_num);

                int i=0;
                for(NodeIt n(_graph); n != INVALID; ++n, ++i){
                    _node_id = i;
                }

                if(_arc_mixing && _node_num > 1){
                    const int skip = std::max(_arc_num / _node_num, 3);
                    int i = 0, j = 0;
                    for(ArcIt a(_graph); a!= INVALID; ++a){
                        _arc_id[a] = i;
                        _source[i] = _node_id[_graph.source(a)];
                        _target[i] = _node_id[_graph.target(a)];
                        if((i += skip) >= _arc_num) i = ++j;
                    }
                }else{
                    int i=0;
                    for(ArcIt a(_graph); a != INVALID; ++a, ++i){
                        _arc_id[a] = i;
                        _source[i] = _node_id[_graph.source(a)];
                        _target[i] = _node_id[_graph.target(a)];
                    }
                }

                return *this;
            }

            template <typename Number>
            Number totalCost() const {
                Number c = 0;
                for(ArcIt a(_graph); a != INVALID; ++a){
                    int i = _arc_id[a];
                    c += Number(_flow[i]) * Number(_cost[i]);
                }
                return c;
            }

            int flow(const Arc &a) const{
                return _flow[_arc_id[a]];
            }

            template <typename FlowMap>
            void flowMap(FlowMap &map) const{
                for(ArcIt a(_graph); a != INVALID; ++a){
                    map.set(a, _flow[_arc_id[a]]);
                }
            }

            int potential(const Node &n) const {
                return _cost_pi[_node_id[n]];
            }

            template <typename PotentialMap>
            void potentialMap(PotentialMap &map) const{
                for(NodeIt n(_graph); n != INVALID; ++n){
                    map.set(n, _cost_pi[_node_id[n]]);
                }
            }

        private:

            bool init(){
                if(_node_num) return false;

                _sum_supply = 0;
                for(int i=0; i!= _node_num; ++i){
                    _sum_supply += _supply[i];
                }

                if(_has_lower) {
                    for(int i=0; i!= _arc_num; ++i){
                        int c = _lower[i];
                        if(c>=0){
                            _cap[i] = _upper[i] <MAX ? _upper[i]-c : INF;
                        }else {
                            _cap[i] = _upper[i] < MAX +c ? _upper[i] - c : INF;
                        }
                        _supply[_source[i]] -= c;
                        _supply[_target[i]] += c;
                    }
                } else {
                    for(int i=0; i!= _arc_num; ++i){
                        _cap[i] = _upper[i];
                    }
                }

                int ART_COST = MAX / 2 + 1;

                for(int i = 0; i != _arc_num; ++i){
                    _flow[i] = 0;
                    _state[i] = STATE_LOWER;
                }

                _root = _node_num;
                _parent[_root] = -1;
                _pred[_root] = -1;
                _thread[_root] = 0;
                _rev_thread[0] = _root;
                _succ_num[_root] = _node_num + 1;
                _last_succ[_root] = _root - 1;
                _supply[_root] = -_sum_supply;
                _cost_pi[_root] = 0;

                if(_sum_supply == 0){
                    _search_arc_num = _arc_num;
                    _all_arc_num = _arc_num + _node_num;
                    for( int u = 0, e = _arc_num; u != _node_num; ++u , ++e){
                        _parent[u] = _root;
                        _pred[u] = e;
                        _thread[u] = u + 1;
                        _rev_thread[u + 1] = u;
                        _succ_num[u] = 1;
                        _last_succ[u] = u;
                        _cap[e] = INF;
                        _state[e] = STATE_TREE;
                        if(_supply[u] >= 0){
                            _pred_dir[u] = DIRECTION_UP;
                            _cost_pi[u] = 0;
                            _source[e] = u;
                            _target[e] = _root;
                            _flow[e] = _supply[u];
                            _cost[e] = 0;
                        }else {
                            _pred_dir[u] = DIRECTION_DOWN;
                            _cost_pi[u] = ART_COST;
                            _source[e] = _root;
                            _target[e] = u;
                            _flow[e] = -_supply[u];
                            _cost[e] = ART_COST;
                        }
                    }
                }
                else if (_sum_supply > 0){
                    _search_arc_num = _arc_num + _node_num;
                    int f = _arc_num + _node_num;
                    for( int u=0, e = _arc_num; u != _node_num; ++u, ++e){
                        _parent[u] = _root;
                        _thread[u] = u + 1;
                        _rev_thread[u + 1] = u;
                        _succ_num[u] = 1;
                        _last_succ[u] = u;
                        if(_supply[u] >= 0){
                            _pred_dir[u] = DIRECTION_UP;
                            _cost_pi[u] = 0;
                            _pred[u] = e;
                            _source[e] = u;
                            _target[e] = _root;
                            _cap[e] = INF;
                            _flow[e] = _supply[u];
                            _cost[e] = 0;
                            _state[e] = STATE_TREE;
                        } else {
                            _pred_dir[u] = DIRECTION_DOWN;
                            _cost_pi[u] = ART_COST;
                            _pred[u] = f;
                            _source[f] = _root;
                            _target[f] = u;
                            _cap[f] = INF;
                            _flow[f] = -_supply[u];
                            _cost[f] = ART_COST;
                            _state[f] =  STATE_TREE;
                            _source[e] = u;
                            _target[e] = _root;
                            _cap[e] = INF;
                            _flow[e] = 0;
                            _cost[e] = 0;
                            _state[e] = STATE_LOWER;
                            ++f;
                        }
                    }
                    _all_arc_num = f;
                }
                else{
                    _search_arc_num = _arc_num + _node_num;
                    int f = _arc_num + _node_num;
                    for( int u=0, e = _arc_num; u != _node_num; ++u, ++e){
                        _parent[u] = _root;
                        _thread[u] = u + 1;
                        _rev_thread[u + 1] = u;
                        _succ_num[u] = 1;
                        _last_succ[u] = u;
                        if(_supply[u] <= 0){
                            _pred_dir[u] = DIRECTION_DOWN;
                            _cost_pi[u] = 0;
                            _pred[u] = e;
                            _source[e] = _root;
                            _target[e] = u;
                            _cap[e] = INF;
                            _flow[e] = -_supply[u];
                            _cost[e] = 0;
                            _state[e] = STATE_TREE;
                        } else {
                            _pred_dir[u] = DIRECTION_UP;
                            _cost_pi[u] = ART_COST;
                            _pred[u] = f;
                            _source[f] = u;
                            _target[f] = _root;
                            _cap[f] = INF;
                            _flow[f] = _supply[u];
                            _cost[f] = ART_COST;
                            _state[f] =  STATE_TREE;
                            _source[e] = _root;
                            _target[e] = u;
                            _cap[e] = INF;
                            _flow[e] = 0;
                            _cost[e] = 0;
                            _state[e] = STATE_LOWER;
                            ++f;
                        }
                    }
                    _all_arc_num = f;
                }

                return true;
            }

            bool checkBoundMaps(){
                for (int j = 0; j != _arc_num; ++j){
                    if(_upper[j] < _lower[j]) return false;
                }
                return true;
            }

            void findJoinNode(){
                int u = _source[in_arc];
                int v = _target[in_arc];
                while ( u != v){
                    if(_succ_num[u] < _succ_num[v]){
                        u = _parent[u];
                    } else {
                        v = _parent[v];
                    }
                }
                join = u;
            }

            bool findLeavingArc(){
                int first, second;
                if(_state[in_arc] == STATE_LOWER){
                    first = _source[in_arc];
                    second = _target[in_arc];
                } else {
                    first = _target[in_arc];
                    second = _source[in_arc];
                }
                delta = _cap[in_arc];
                int result = 0;
                int c,d,e;

                for(int u = first; u != join; u = _parent[u]){
                    e = _pred[u];
                    d = _flow[e];
                    if(_pred_dir[u] == DIRECTION_DOWN){
                        c = _cap[e];
                        d = c >= MAX ? INF : c-d;
                    }
                    if(d < delta) {
                        delta = d;
                        u_out = u;
                        result = 1;
                    }
                }

                for(int u = second; u != join; u = _parent[u]){
                    e = _pred[u];
                    d = _flow[e];
                    if(_pred_dir[u] == DIRECTION_DOWN){
                        c = _cap[e];
                        d = c >= MAX ? INF : c-d;
                    }
                    if(d < delta) {
                        delta = d;
                        u_out = u;
                        result = 2;
                    }
                }

                if(result == 1){
                    u_in = first;
                    v_in = second;
                } else {
                    u_in = second;
                    v_in = first;
                }
                return result != 0;
            }

            void changeFlow(bool change){
                if(delta > 0){
                    int val = _state[in_arc] * delta;
                    _flow[in_arc] += val;
                    for(int u = _source[in_arc]; u!= join; u = _parent[u]){
                        _flow[_pred[u]] -= _pred_dir[u] * val;
                    }
                    for(int u = _target[in_arc]; u!= join; u = _parent[u]){
                        _flow[_pred[u]] -= _pred_dir[u] * val;
                    }
                }
                if(change){
                    _state[in_arc] = STATE_TREE;
                    _state[_pred[u_out]] = (_flow[_pred[u_out]] == 0) ? STATE_LOWER : STATE_UPPER;
                }else {
                    _state[in_arc] = -_state[in_arc];
                }
            }

            void updateTreeStructure() {
                int old_rev_thread = _rev_thread[u_out];
                int old_succ_num = _succ_num[u_out];
                int old_last_succ = _last_succ[u_out];
                v_out = _parent[u_out];

                // Check if u_in and u_out coincide
                if (u_in == u_out) {
                    // Update _parent, _pred, _pred_dir
                    _parent[u_in] = v_in;
                    _pred[u_in] = in_arc;
                    _pred_dir[u_in] = u_in == _source[in_arc] ? DIRECTION_UP : DIRECTION_DOWN;

                    // Update _thread and _rev_thread
                    if (_thread[v_in] != u_out) {
                        int after = _thread[old_last_succ];
                        _thread[old_rev_thread] = after;
                        _rev_thread[after] = old_rev_thread;
                        after = _thread[v_in];
                        _thread[v_in] = u_out;
                        _rev_thread[u_out] = v_in;
                        _thread[old_last_succ] = after;
                        _rev_thread[after] = old_last_succ;
                    }
                } else {
                    // Handle the case when old_rev_thread equals to v_in
                    // (it also means that join and v_out coincide)
                    int thread_continue = old_rev_thread == v_in ?
                                          _thread[old_last_succ] : _thread[v_in];

                    // Update _thread and _parent along the stem nodes (i.e. the nodes
                    // between u_in and u_out, whose parent have to be changed)
                    int stem = u_in;              // the current stem node
                    int par_stem = v_in;          // the new parent of stem
                    int next_stem;                // the next stem node
                    int last = _last_succ[u_in];  // the last successor of stem
                    int before, after = _thread[last];
                    _thread[v_in] = u_in;
                    _dirty_revs.clear();
                    _dirty_revs.push_back(v_in);
                    while (stem != u_out) {
                        // Insert the next stem node into the thread list
                        next_stem = _parent[stem];
                        _thread[last] = next_stem;
                        _dirty_revs.push_back(last);

                        // Remove the subtree of stem from the thread list
                        before = _rev_thread[stem];
                        _thread[before] = after;
                        _rev_thread[after] = before;

                        // Change the parent node and shift stem nodes
                        _parent[stem] = par_stem;
                        par_stem = stem;
                        stem = next_stem;

                        // Update last and after
                        last = _last_succ[stem] == _last_succ[par_stem] ?
                               _rev_thread[par_stem] : _last_succ[stem];
                        after = _thread[last];
                    }
                    _parent[u_out] = par_stem;
                    _thread[last] = thread_continue;
                    _rev_thread[thread_continue] = last;
                    _last_succ[u_out] = last;

                    // Remove the subtree of u_out from the thread list except for
                    // the case when old_rev_thread equals to v_in
                    if (old_rev_thread != v_in) {
                        _thread[old_rev_thread] = after;
                        _rev_thread[after] = old_rev_thread;
                    }

                    // Update _rev_thread using the new _thread values
                    for (int i = 0; i != int(_dirty_revs.size()); ++i) {
                        int u = _dirty_revs[i];
                        _rev_thread[_thread[u]] = u;
                    }

                    // Update _pred, _pred_dir, _last_succ and _succ_num for the
                    // stem nodes from u_out to u_in
                    int tmp_sc = 0, tmp_ls = _last_succ[u_out];
                    for (int u = u_out, p = _parent[u]; u != u_in; u = p, p = _parent[u]) {
                        _pred[u] = _pred[p];
                        _pred_dir[u] = -_pred_dir[p];
                        tmp_sc += _succ_num[u] - _succ_num[p];
                        _succ_num[u] = tmp_sc;
                        _last_succ[p] = tmp_ls;
                    }
                    _pred[u_in] = in_arc;
                    _pred_dir[u_in] = u_in == _source[in_arc] ? DIRECTION_UP : DIRECTION_DOWN;
                    _succ_num[u_in] = old_succ_num;
                }

                // Update _last_succ from v_in towards the root
                int up_limit_out = _last_succ[join] == v_in ? join : -1;
                int last_succ_out = _last_succ[u_out];
                for (int u = v_in; u != -1 && _last_succ[u] == v_in; u = _parent[u]) {
                    _last_succ[u] = last_succ_out;
                }

                // Update _last_succ from v_out towards the root
                if (join != old_rev_thread && v_in != old_rev_thread) {
                    for (int u = v_out; u != up_limit_out && _last_succ[u] == old_last_succ;
                         u = _parent[u]) {
                        _last_succ[u] = old_rev_thread;
                    }
                }
                else if (last_succ_out != old_last_succ) {
                    for (int u = v_out; u != up_limit_out && _last_succ[u] == old_last_succ;
                         u = _parent[u]) {
                        _last_succ[u] = last_succ_out;
                    }
                }

                // Update _succ_num from v_in to join
                for (int u = v_in; u != join; u = _parent[u]) {
                    _succ_num[u] += old_succ_num;
                }
                // Update _succ_num from v_out to join
                for (int u = v_out; u != join; u = _parent[u]) {
                    _succ_num[u] -= old_succ_num;
                }
            }

            void updatePotential() {
                int sigma = _cost_pi[v_in] - _cost_pi[u_in] -
                             _pred_dir[u_in] * _cost[in_arc];
                int end = _thread[_last_succ[u_in]];
                for (int u = u_in; u != end; u = _thread[u]) {
                    _cost_pi[u] += sigma;
                }
            }

            // Heuristic initial pivots
            bool initialPivots() {
                int curr, total = 0;
                std::vector<Node> supply_nodes, demand_nodes;
                for (NodeIt u(_graph); u != INVALID; ++u) {
                    curr = _supply[_node_id[u]];
                    if (curr > 0) {
                        total += curr;
                        supply_nodes.push_back(u);
                    }
                    else if (curr < 0) {
                        demand_nodes.push_back(u);
                    }
                }
                if (_sum_supply > 0) total -= _sum_supply;
                if (total <= 0) return true;

                IntVector arc_vector;
                if (_sum_supply >= 0) {
                    if (supply_nodes.size() == 1 && demand_nodes.size() == 1) {
                        // Perform a reverse graph search from the sink to the source
                        typename G::template NodeMap<bool> reached(_graph, false);
                        Node s = supply_nodes[0], t = demand_nodes[0];
                        std::vector<Node> stack;
                        reached[t] = true;
                        stack.push_back(t);
                        while (!stack.empty()) {
                            Node u, v = stack.back();
                            stack.pop_back();
                            if (v == s) break;
                            for (InArcIt a(_graph, v); a != INVALID; ++a) {
                                if (reached[u = _graph.source(a)]) continue;
                                int j = _arc_id[a];
                                if (_cap[j] >= total) {
                                    arc_vector.push_back(j);
                                    reached[u] = true;
                                    stack.push_back(u);
                                }
                            }
                        }
                    } else {
                        // Find the min. cost incoming arc for each demand node
                        for (int i = 0; i != int(demand_nodes.size()); ++i) {
                            Node v = demand_nodes[i];
                            int c, min_cost = std::numeric_limits<int>::max();
                            Arc min_arc = INVALID;
                            for (InArcIt a(_graph, v); a != INVALID; ++a) {
                                c = _cost[_arc_id[a]];
                                if (c < min_cost) {
                                    min_cost = c;
                                    min_arc = a;
                                }
                            }
                            if (min_arc != INVALID) {
                                arc_vector.push_back(_arc_id[min_arc]);
                            }
                        }
                    }
                } else {
                    // Find the min. cost outgoing arc for each supply node
                    for (int i = 0; i != int(supply_nodes.size()); ++i) {
                        Node u = supply_nodes[i];
                        int c, min_cost = std::numeric_limits<int>::max();
                        Arc min_arc = INVALID;
                        for (OutArcIt a(_graph, u); a != INVALID; ++a) {
                            c = _cost[_arc_id[a]];
                            if (c < min_cost) {
                                min_cost = c;
                                min_arc = a;
                            }
                        }
                        if (min_arc != INVALID) {
                            arc_vector.push_back(_arc_id[min_arc]);
                        }
                    }
                }

                // Perform heuristic initial pivots
                for (int i = 0; i != int(arc_vector.size()); ++i) {
                    in_arc = arc_vector[i];
                    if (_state[in_arc] * (_cost[in_arc] + _cost_pi[_source[in_arc]] -
                                          _cost_pi[_target[in_arc]]) >= 0) continue;
                    findJoinNode();
                    bool change = findLeavingArc();
                    if (delta >= MAX) return false;
                    changeFlow(change);
                    if (change) {
                        updateTreeStructure();
                        updatePotential();
                    }
                }
                return true;
            }

            ProblemType start(PivotRule pivot_rule) {
                // Select the pivot rule implementation
                switch (pivot_rule) {
                    case FIRST_ELIGIBLE:
                        return start<FirstSearch>();
                }
                return INFEASIBLE; // avoid warning
            }

            template <typename PivotRuleImpl>
            ProblemType start() {
                PivotRuleImpl pivot(*this);

                // Perform heuristic initial pivots
                if (!initialPivots()) return UNBOUNDED;

                // Execute the Network Simplex algorithm
                while (pivot.findEnteringArc()) {
                    findJoinNode();
                    bool change = findLeavingArc();
                    if (delta >= MAX) return UNBOUNDED;
                    changeFlow(change);
                    if (change) {
                        updateTreeStructure();
                        updatePotential();
                    }
                }

                // Check feasibility
                for (int e = _search_arc_num; e != _all_arc_num; ++e) {
                    if (_flow[e] != 0) return INFEASIBLE;
                }

                // Transform the solution and the supply map to the original form
                if (_has_lower) {
                    for (int i = 0; i != _arc_num; ++i) {
                        int c = _lower[i];
                        if (c != 0) {
                            _flow[i] += c;
                            _supply[_source[i]] += c;
                            _supply[_target[i]] -= c;
                        }
                    }
                }

                if (_sum_supply == 0) {
//                    if (_stype == GEQ) {
//                        int max_pot = -std::numeric_limits<int>::max();
//                        for (int i = 0; i != _node_num; ++i) {
//                            if (_cost_pi[i] > max_pot) max_pot = _cost_pi[i];
//                        }
//                        if (max_pot > 0) {
//                            for (int i = 0; i != _node_num; ++i)
//                                _cost_pi[i] -= max_pot;
//                        }
//                    } else {
                        int min_pot = std::numeric_limits<int>::max();
                        for (int i = 0; i != _node_num; ++i) {
                            if (_cost_pi[i] < min_pot) min_pot = _cost_pi[i];
                        }
                        if (min_pot < 0) {
                            for (int i = 0; i != _node_num; ++i)
                                _cost_pi[i] -= min_pot;
                        }
//                    }
                }
                return OPTIMAL;
            }
        };
    }
}

#endif //EFFICIENTGRAPH_LIB_SOLVER_H
