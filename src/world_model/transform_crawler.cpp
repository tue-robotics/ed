#include "ed/world_model/transform_crawler.h"

#include "ed/world_model.h"
#include "ed/entity.h"
#include "ed/relation.h"

namespace ed
{
namespace world_model
{

// ----------------------------------------------------------------------------------------------------

TransformCrawler::TransformCrawler(const WorldModel& wm, const UUID& root_id, const Time& time)
    : wm_(wm), time_(time)
{
    Idx root_idx;
    if (wm_.findEntityIdx(root_id, root_idx))
    {
        const EntityConstPtr& e = wm_.entities()[root_idx];
        pushChildren(*e, geo::Pose3D::identity());
        visited_.insert(root_idx);
    }
    else
    {
        std::cout << "NOT wm_.findEntityIdx(" << root_id << ", " << root_idx << ")" << std::endl;
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
    //std::cout << "TransformCrawler queue leng: " << queue_.size() << std::endl;
    if (queue_.empty())
        return false;

    const ed::EntityConstPtr& e = entity();
    if (!e)
        return false;

    pushChildren(*e, transform());

    std::cout << "TransformCrawler::next queue_.size() = " << queue_.size() << std::endl;

    queue_.pop();

    return true;
}

// ----------------------------------------------------------------------------------------------------

void TransformCrawler::pushChildren(const Entity& e, const geo::Pose3D& transform)
{
    // Push all nodes that point to this node
    const std::map<Idx, Idx>& transforms_to = e.relationsTo();

    std::cout << e.id() << " has " << transforms_to.size() << " relations to other entities" << std::endl;

    for(std::map<Idx, Idx>::const_iterator it = transforms_to.begin(); it != transforms_to.end(); ++it)
    {
        Idx n2 = it->first;
        if (visited_.find(n2) == visited_.end())
        {
            geo::Pose3D rel_transform;
            RelationConstPtr r = wm_.relations()[it->second];
            std::cout << "TO: " << n2 << "has a "<< typeid(r).name() << " relation r " << r << " with " << it->second << std::endl;
            if (r && r->calculateTransform(time_, rel_transform))
            {
                std::cout << "TO: " << n2 << ": transform * rel_transform = " << transform << " * " << rel_transform << " = " << transform * rel_transform << std::endl;
                queue_.push(Node(n2, transform * rel_transform));
            }
            else
            {
                std::cout << "TO: NOT (r && r->calculateTransform(time_, rel_transform))" << std::endl;
            }

            visited_.insert(n2);
            std::cout << "TO: Inserted " << n2 << " in visited_. Has " << visited_.size() << " elements now" << std::endl;
        }
    }

    // Push all nodes this node points to
    const std::map<Idx, Idx>& transforms_from = e.relationsFrom();
    for(std::map<Idx, Idx>::const_iterator it = transforms_from.begin(); it != transforms_from.end(); ++it)
    {
        Idx n2 = it->first;
        if (visited_.find(n2) == visited_.end())
        {
            geo::Pose3D rel_transform;
            RelationConstPtr r = wm_.relations()[it->second];
            std::cout << "FROM: " << n2 << "has a "<< typeid(r).name() << " relation r " << r << " with " << it->second << std::endl;
            if (r && r->calculateTransform(time_, rel_transform))
            {
                std::cout << "FROM: " << n2 << ": transform * rel_transform.inverse() = " << transform << " * " << rel_transform.inverse() << " = " << transform * rel_transform.inverse() << std::endl;
                queue_.push(Node(n2, transform * rel_transform.inverse()));
            }
            else
            {
                std::cout << "FROM: NOT (r && r->calculateTransform(time_, rel_transform))" << std::endl;
            }

            visited_.insert(n2);
            std::cout << "FROM: Inserted " << n2 << " in visited_. Has " << visited_.size() << " elements now" << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

} // end namespace ed

} // end namespace world_model

