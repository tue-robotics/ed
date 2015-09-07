#ifndef ED_WORLD_MODEL_H_
#define ED_WORLD_MODEL_H_

#include "ed/types.h"
#include "ed/time.h"

#include <geolib/datatypes.h>

#include <queue>

namespace ed
{

class PropertyKeyDB;
class PropertyKeyDBEntry;

// ----------------------------------------------------------------------------------------------------

class WorldModel
{

public:

    class EntityIterator : public std::iterator<std::forward_iterator_tag, EntityConstPtr>
    {

    public:

        EntityIterator(const std::vector<EntityConstPtr>& v) : it_(v.begin()), it_end_(v.end())
        {
            // Skip possible zero-entities (deleted entities) at the beginning
            while(it_ != it_end_ && !(*it_))
                ++it_;
        }

        EntityIterator(const EntityIterator& it) : it_(it.it_) {}

        EntityIterator(const std::vector<EntityConstPtr>::const_iterator& it) : it_(it) {}

        EntityIterator& operator++()
        {
            // Increase iterator and skip possible zero-entities (deleted entities)
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

    WorldModel(const PropertyKeyDB* prop_key_db = 0);

    inline const_iterator begin() const { return const_iterator(entities_); }

    inline const_iterator end() const { return const_iterator(entities_.end()); }

    void setEntity(const UUID& id, const EntityConstPtr& e);

    void removeEntity(const UUID& id);

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

    void setRelation(Idx parent, Idx child, const RelationConstPtr& r);

    bool findEntityIdx(const UUID& id, Idx& idx) const;

    bool calculateTransform(const UUID& source, const UUID& target, const Time& time, geo::Pose3D& tf) const;

    /// Warning: the return vector may return null-pointers
    const std::vector<EntityConstPtr>& entities() const { return entities_; }

    /// Warning: the return vector may return null-pointers
    const std::vector<RelationConstPtr>& relations() const { return relations_; }

    unsigned long revision() const { return revision_; }

    const std::vector<unsigned long>& entity_revisions() const { return entity_revisions_; }

    const std::vector<unsigned long>& entity_shape_revisions() const { return entity_shape_revisions_; }

    const PropertyKeyDBEntry* getPropertyInfo(const std::string& name) const;

private:

    unsigned long revision_;

    std::map<UUID, Idx> entity_map_;

    std::vector<EntityConstPtr> entities_;

    std::vector<unsigned long> entity_revisions_;

    std::vector<unsigned long> entity_shape_revisions_;

    std::queue<Idx> entity_empty_spots_;

    std::vector<RelationConstPtr> relations_;

    const PropertyKeyDB* property_info_db_;

    Idx addRelation(const RelationConstPtr& r);

    EntityPtr getOrAddEntity(const UUID& id, std::map<UUID, EntityPtr>& new_entities);

    Idx addNewEntity(const EntityConstPtr& e);


};

}

#endif
