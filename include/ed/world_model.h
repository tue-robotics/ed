#ifndef ED_WORLD_MODEL_H_
#define ED_WORLD_MODEL_H_

// Struct holding the world model data

#include "types.h"

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

    typedef std::map<UUID, EntityConstPtr>::iterator iterator;

    typedef std::map<UUID, EntityConstPtr>::const_iterator const_iterator;

    iterator begin() { return entity_map_.begin(); }

    const_iterator begin() const { return entity_map_.begin(); }

    iterator end() { return entity_map_.end(); }

    const_iterator end() const { return entity_map_.end(); }

    void setEntity(const UUID& id, const EntityConstPtr& e);

    void removeEntity(const UUID& id);

    EntityConstPtr getEntity(const ed::UUID& id) const
    {
        std::map<ed::UUID, ed::EntityConstPtr>::const_iterator it = entity_map_.find(id);
        if (it == entity_map_.end())
            return EntityConstPtr();
        else
            return it->second;
    }

    size_t numEntities() const { return entity_map_.size(); }

    void update(const UpdateRequest& req);

    void setRelation(Idx parent, Idx child, const RelationPtr& r);

private:

    std::map<UUID, EntityConstPtr> entity_map_;

    std::vector<EntityConstPtr> entities_;

    std::vector<RelationConstPtr> relations_;

    Idx addRelation(const RelationPtr& r);

};

}

#endif
