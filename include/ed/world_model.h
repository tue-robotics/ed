#ifndef ED_WORLD_MODEL_H_
#define ED_WORLD_MODEL_H_

// Struct holding the world model data

#include "types.h"

#include <queue>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

class Relation
{

private:

    Idx parent_idx_;
    Idx child_idx_;

};

// ----------------------------------------------------------------------------------------------------

class RelationMap
{

public:

    std::map<Idx, Idx> relations;

};

// ----------------------------------------------------------------------------------------------------

class WorldModel
{

public:

    class EntityIterator : public std::iterator<std::forward_iterator_tag, EntityConstPtr>
    {

    public:

        EntityIterator(const std::vector<EntityConstPtr>& v) : it_(v.begin()), it_end_(v.end()) {}

        EntityIterator(const EntityIterator& it) : it_(it.it_) {}

        EntityIterator(const std::vector<EntityConstPtr>::const_iterator& it) : it_(it) {}

        EntityIterator& operator++()
        {
            do { ++it_; if (it_ == it_end_) break; } while (!(*it_));
            return *this;
        }

        EntityIterator operator++(int) { EntityIterator tmp(*this); operator++(); return tmp; }

        bool operator==(const EntityIterator& rhs) { return it_ == rhs.it_; }

        bool operator!=(const EntityIterator& rhs) { return it_ != rhs.it_; }

        const EntityConstPtr& operator*() { return *it_; }

    private:

        std::vector<EntityConstPtr>::const_iterator it_;
        std::vector<EntityConstPtr>::const_iterator it_end_;

    };

    typedef EntityIterator const_iterator;

    const_iterator begin() const { return const_iterator(entities_); }

    const_iterator end() const { return const_iterator(entities_.end()); }

    void setEntity(const UUID& id, const EntityConstPtr& e);

    void removeEntity(const UUID& id);

    EntityConstPtr getEntity(const ed::UUID& id) const
    {
        std::map<ed::UUID, Idx>::const_iterator it = entity_map_.find(id);
        if (it == entity_map_.end())
            return EntityConstPtr();
        else
            return entities_[it->second];
    }

    size_t numEntities() const { return entity_map_.size(); }

    void update(const UpdateRequest& req);

    void setRelation(Idx parent, Idx child, const RelationPtr& r);

private:

    std::map<UUID, Idx> entity_map_;

    std::vector<EntityConstPtr> entities_;

    std::queue<Idx> entity_empty_spots_;

    std::vector<RelationConstPtr> relations_;

    Idx addRelation(const RelationPtr& r);

    EntityPtr getOrAddEntity(const UUID& id, std::map<UUID, EntityPtr>& new_entities);

    void addNewEntity(const EntityConstPtr& e);


};

}

#endif
