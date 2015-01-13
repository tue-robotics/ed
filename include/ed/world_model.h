#ifndef ED_WORLD_MODEL_H_
#define ED_WORLD_MODEL_H_

#include "ed/types.h"
#include "ed/time.h"

#include <geolib/datatypes.h>

#include <queue>

namespace ed
{

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

    WorldModel();

    typedef EntityIterator const_iterator;

    inline const_iterator begin() const { return const_iterator(entities_); }

    inline const_iterator end() const { return const_iterator(entities_.end()); }

    void setEntity(const UUID& id, const EntityConstPtr& e);

    void removeEntity(const UUID& id);

    const EntityConstPtr& getEntity(Idx idx) const { return entities_[idx]; }

    EntityConstPtr getEntity(const ed::UUID& id) const
    {
        Idx idx;
        if (findEntityIdx(id, idx))
            return entities_[idx];
        else
            return EntityConstPtr();
    }

    size_t numEntities() const { return entity_map_.size(); }

    void update(const UpdateRequest& req);

    RelationConstPtr getRelation(Idx idx) const { return relations_[idx]; }

    bool findEntityIdx(const UUID& id, Idx& idx) const;

    bool calculateTransform(const UUID& source, const UUID& target, const Time& time, geo::Pose3D& tf) const;

    const Time& latestTime() const { return latest_time_; }

private:

    // Timestamp of most recent update
    Time latest_time_;

    std::map<UUID, Idx> entity_map_;

    std::vector<EntityConstPtr> entities_;

    std::queue<Idx> entity_empty_spots_;

    std::vector<RelationConstPtr> relations_;

    Idx addRelation(const RelationConstPtr& r);

    EntityPtr getOrAddEntity(const UUID& id, std::map<UUID, EntityPtr>& new_entities);

    void addNewEntity(const EntityConstPtr& e);

    void setRelation(const EntityPtr& p, const EntityPtr& c, const RelationConstPtr& r);

};

}

#endif
