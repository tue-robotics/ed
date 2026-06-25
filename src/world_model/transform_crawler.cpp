#include "ed/world_model/transform_crawler.h"

#include "ed/entity.h"
#include "ed/relation.h"
#include "ed/time.h"
#include "ed/types.h"
#include "ed/uuid.h"
#include "ed/world_model.h"
#include <geolib/datatypes.h>
#include <map>

namespace ed::world_model
{

// ----------------------------------------------------------------------------------------------------

TransformCrawler::TransformCrawler(const WorldModel& wm, const UUID& root_id, const Time& time) : wm_(wm), time_(time)
{
    Idx root_idx = 0;
    if (wm_.findEntityIdx(root_id, root_idx))
    {
        const EntityConstPtr& e = wm_.entities()[root_idx];
        pushChildren(*e, geo::Pose3D::identity());
        visited_.insert(root_idx);
    }
}

// ----------------------------------------------------------------------------------------------------

const EntityConstPtr& TransformCrawler::entity() const
{
    return wm_.entities()[queue_.front().entity_idx];
}

// ----------------------------------------------------------------------------------------------------

bool TransformCrawler::next()
{
    if (queue_.empty())
        return false;

    const ed::EntityConstPtr& e = entity();
    if (!e)
        return false;

    pushChildren(*e, transform());

    queue_.pop();

    return true;
}

// ----------------------------------------------------------------------------------------------------

void TransformCrawler::pushChildren(const Entity& e, const geo::Pose3D& transform)
{
    // Push all nodes that point to this node
    const std::map<Idx, Idx>& transforms_to = e.relationsTo();
    for (auto it : transforms_to)
    {
        Idx const n2 = it.first;
        if (visited_.find(n2) == visited_.end())
        {
            geo::Pose3D rel_transform;
            RelationConstPtr const r = wm_.relations()[it.second];
            if (r && r->calculateTransform(time_, rel_transform))
                queue_.emplace(n2, transform * rel_transform);

            visited_.insert(n2);
        }
    }

    // Push all nodes this node points to
    const std::map<Idx, Idx>& transforms_from = e.relationsFrom();
    for (auto it : transforms_from)
    {
        Idx const n2 = it.first;
        if (visited_.find(n2) == visited_.end())
        {
            geo::Pose3D rel_transform;
            RelationConstPtr const r = wm_.relations()[it.second];
            if (r && r->calculateTransform(time_, rel_transform))
                queue_.emplace(n2, transform * rel_transform.inverse());

            visited_.insert(n2);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

} // namespace ed::world_model
