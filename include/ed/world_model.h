#ifndef ED_WORLD_MODEL_H_
#define ED_WORLD_MODEL_H_

// Struct holding the world model data

#include "types.h"

namespace ed
{

class WorldModel
{

public:

    typedef std::map<UUID, EntityConstPtr>::iterator iterator;

    typedef std::map<UUID, EntityConstPtr>::const_iterator const_iterator;

    iterator begin() { return entities_.begin(); }

    const_iterator begin() const { return entities_.begin(); }

    iterator end() { return entities_.end(); }

    const_iterator end() const { return entities_.end(); }

    void setEntities(const std::map<UUID, EntityConstPtr>& entities) { entities_ = entities; }

    EntityConstPtr getEntity(const ed::UUID& id) const
    {
        std::map<ed::UUID, ed::EntityConstPtr>::const_iterator it = entities_.find(id);
        if (it == entities_.end())
            return EntityConstPtr();
        else
            return it->second;
    }

private:

    std::map<UUID, EntityConstPtr> entities_;

};

}

#endif
