//
// Created by huyang on 17-3-10.
//

#ifndef EFFICIENTGRAPH_LIB_MAP_H
#define EFFICIENTGRAPH_LIB_MAP_H

namespace base{
    struct True{ static const bool value = true; };
    struct False { static const bool value = false; };
    //----------------------------Readable map-------------------------
    template <typename K, typename V>
    class ReadMap
    {
    public:
        typedef K Key;
        typedef V Value;

        Value operator[](const Key &) const{
            return *(static_cast<Value *>(0)+1);
        }
    };

    //----------------------------Writable map-------------------------
    template <typename K, typename V>
    class WriteMap
    {
    public:
        typedef K Key;
        typedef V Value;

        void set(const Key &, const Value &){}
        WriteMap() {}
    };

    //--------------------------Read/Write map--------------------------
    template <typename K, typename V>
    class ReadWriteMap : public ReadMap<K,V>, public WriteMap<K,V>
    {
    public:
        typedef K Key;
        typedef V Value;

        Value operator[](const Key &) const{
            Value  *r = 0;
            return *r;
        }
        void set(const Key &, const Value &){}
        ReadWriteMap() {}
    };

    //---------------------------Referable map---------------------------
    template <typename K, typename V, typename R, typename CR>
    class ReferenceMap : public ReadWriteMap<K,V>
    {
    public:
        typedef True ReferenceMapTag;
        typedef K Key;
        typedef V Value;
        typedef R Reference;
        typedef CR ConstReference;

        Reference operator[](const Key &){
            Value *r = 0;
            return *r;
        }

        ConstReference operator[](const Key &) const{
            Value *r = 0;
            return *r;
        }

        void set(const Key &k,const Value &v) { operator[](k)=v; }
    };



    //---------------------------Map Extender----------------------------
    template <typename MapT>
    class MapExtender : public MapT{
        typedef MapT Parent;
        typedef typedef Parent::GraphType GraphType;
    public:
        typedef MapExtender Map;
        typedef typename Parent::Key Item;
        typedef typename Parent::Key Key;
        typedef typename Parent::Value Value;
        typedef typename Parent::Reference Reference;
        typedef typename Parent::ConstReference ConstReference;
        typedef typename Parent::ReferenceMapTag ReferenceMapTag;

        class MapIt;
        class ConstMapIt;

        friend class MapIt;
        friend class ConstMapIt;

        MapExtender(const GraphType & graph) : Parent(graph) {}
        MapExtender(const GraphType & graph, const Value & value) : Parent(graph, value) {}

    private:
        MapExtender &operator=(const MapExtender &cmap) { return operator=<MapExtender>(cmap); }
        template <typename CMap>
        MapExtender &operator=(const CMap & cmap) { return Parent::operator=(cmap); return *this; }

    public:
        class MapIt : public Item{
            typedef Item Parent;

        protected:
            map;

        public:
            typedef typename Map::Value Value;
            MapIt() : map(NULL) {}
            MapIt(Invalid i) : Parent (i), map(NULL) {}

            explicit MapIt(Map &_map) : map(&_map){

            }
        };
    };
}

#endif //EFFICIENTGRAPH_MAP_LIB_H
