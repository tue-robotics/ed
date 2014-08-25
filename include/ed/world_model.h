#ifndef ED_WORLD_MODEL_H_
#define ED_WORLD_MODEL_H_

// Struct holding the world model data

#include "types.h"

namespace ed
{

class WorldModel
{

public:

    typedef std::map<UUID, EntityConstPtr>::iterator entity_iterator;

    typedef std::map<UUID, EntityConstPtr>::const_iterator entity_const_iterator;

    entity_iterator begin() { return entities_.begin(); }

    entity_const_iterator begin() const { return entities_.begin(); }

    entity_iterator end() { return entities_.end(); }

    entity_const_iterator end() const { return entities_.end(); }

    void setEntities(const std::map<UUID, EntityConstPtr>& entities) { entities_ = entities; }

private:

    std::map<UUID, EntityConstPtr> entities_;

};

}

#endif
