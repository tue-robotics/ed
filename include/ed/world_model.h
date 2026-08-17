#ifndef ED_WORLD_MODEL_H_
#define ED_WORLD_MODEL_H_

#include "ed/time.h"
#include "ed/types.h"
#include <cstdint>

#include <geolib/datatypes.h>

#include <cstddef>
#include <iterator>
#include <map>
#include <queue>
#include <vector>

namespace ed
{

class PropertyKeyDB;
struct PropertyKeyDBEntry;

// ----------------------------------------------------------------------------------------------------

class WorldModel
{

public:
    class EntityIterator
    {

    public:
        // std::iterator is deprecated in C++17; spell out the traits it used to provide.
        using iterator_category = std::forward_iterator_tag;
        using value_type = EntityConstPtr;
        using difference_type = std::ptrdiff_t;
        using pointer = const EntityConstPtr*;
        using reference = const EntityConstPtr&;

        explicit EntityIterator(const std::vector<EntityConstPtr>& v) : it_(v.begin()), it_end_(v.end())
        {
            // Skip possible zero-entities (deleted entities) at the beginning
            while (it_ != it_end_ && !(*it_))
                ++it_;
        }

        EntityIterator(const EntityIterator& it) : it_(it.it_) {}

        explicit EntityIterator(const std::vector<EntityConstPtr>::const_iterator& it) : it_(it) {}

        EntityIterator& operator++()
        {
            // Increase iterator and skip possible zero-entities (deleted entities)
            ++it_;
            while (it_ != it_end_ && !(*it_))
                ++it_;
            return *this;
        }

        EntityIterator operator++(int)
        {
            EntityIterator const tmp(*this);
            operator++();
            return tmp;
        }

        bool operator==(const EntityIterator& rhs) { return it_ == rhs.it_; }

        bool operator!=(const EntityIterator& rhs) { return it_ != rhs.it_; }

        const EntityConstPtr& operator*() { return *it_; }

    private:
        std::vector<EntityConstPtr>::const_iterator it_;
        std::vector<EntityConstPtr>::const_iterator it_end_;
    };

    using const_iterator = EntityIterator;

    explicit WorldModel(const PropertyKeyDB* prop_key_db = nullptr);

    [[nodiscard]]
    const_iterator begin() const
    {
        return const_iterator{entities_};
    }

    [[nodiscard]]
    const_iterator end() const
    {
        return const_iterator{entities_.end()};
    }

    void setEntity(const UUID& id, const EntityConstPtr& e);

    void removeEntity(const UUID& id);

    [[nodiscard]]
    EntityConstPtr getEntity(const ed::UUID& id) const
    {
        Idx idx = 0;
        if (findEntityIdx(id, idx))
            return entities_[idx];
        return {};
    }

    [[nodiscard]]
    size_t numEntities() const
    {
        return entity_map_.size();
    }

    void update(const UpdateRequest& req);

    void setRelation(Idx parent, Idx child, const RelationConstPtr& r);

    bool findEntityIdx(const UUID& id, Idx& idx) const;

    bool calculateTransform(const UUID& source, const UUID& target, const Time& time, geo::Pose3D& tf) const;

    /// Warning: the return vector may return null-pointers
    [[nodiscard]]
    const std::vector<EntityConstPtr>& entities() const
    {
        return entities_;
    }

    /// Warning: the return vector may return null-pointers
    [[nodiscard]]
    const std::vector<RelationConstPtr>& relations() const
    {
        return relations_;
    }

    [[nodiscard]]
    std::uint64_t revision() const
    {
        return revision_;
    }

    [[nodiscard]]
    const std::vector<std::uint64_t>& entityRevisions() const
    {
        return entity_revisions_;
    }

    [[nodiscard]] [[deprecated(
        "Use entityVisualRevisions(), entityCollisionRevisions() or entityVolumesRevisions() instead.")]]
    const std::vector<std::uint64_t>& entityShapeRevisions() const
    {
        return entityVisualRevisions();
    }

    [[nodiscard]]
    const std::vector<std::uint64_t>& entityVisualRevisions() const
    {
        return entity_visual_revisions_;
    }

    [[nodiscard]]
    const std::vector<std::uint64_t>& entityCollisionRevisions() const
    {
        return entity_collision_revisions_;
    }

    [[nodiscard]]
    const std::vector<std::uint64_t>& entityVolumesRevisions() const
    {
        return entity_volumes_revisions_;
    }

    [[nodiscard]]
    const PropertyKeyDBEntry* getPropertyInfo(const std::string& name) const;

private:
    std::uint64_t revision_{0};

    std::map<UUID, Idx> entity_map_;

    std::vector<EntityConstPtr> entities_;

    std::vector<std::uint64_t> entity_revisions_;

    std::vector<std::uint64_t> entity_visual_revisions_;

    std::vector<std::uint64_t> entity_collision_revisions_;

    std::vector<std::uint64_t> entity_volumes_revisions_;

    std::queue<Idx> entity_empty_spots_;

    std::vector<RelationConstPtr> relations_;

    const PropertyKeyDB* property_info_db_;

    Idx addRelation(const RelationConstPtr& r);

    EntityPtr getOrAddEntity(const UUID& id, std::map<UUID, EntityPtr>& new_entities);

    Idx addNewEntity(const EntityConstPtr& e);
};

} // namespace ed

#endif
