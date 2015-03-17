#ifndef ED_WORLD_MODEL_TRANSFORM_CRAWLER_H_
#define ED_WORLD_MODEL_TRANSFORM_CRAWLER_H_

#include "ed/types.h"
#include "ed/time.h"
#include <geolib/datatypes.h>

#include <queue>

namespace ed
{
namespace world_model
{

/**
 * @brief The TransformCrawler class
 *
 * Starting with the given entity ID, this class crawls the world model transformation graph in a breadth-first manner.
 * Each node (world model entity) is only visited once. For each visited entity, the transformation from the root entity
 * (i.e., the given entity) is calculated.
 */
class TransformCrawler
{

    struct Node
    {
        Node(Idx entity_idx_, const geo::Pose3D& transform_)
            : entity_idx(entity_idx_), transform(transform_) {}
        Idx entity_idx;
        geo::Pose3D transform;
    };

public:

    TransformCrawler(const WorldModel& wm, const UUID& start_id, const Time& time);

    bool next();

    bool hasNext() const { return !queue_.empty(); }

    const geo::Pose3D& transform() const { return queue_.front().transform; }

    const EntityConstPtr& entity() const;

private:

    const WorldModel& wm_;

    Time time_;

    std::set<Idx> visited_;

    std::queue<Node> queue_;

    void pushChildren(const Entity& e, const geo::Pose3D& transform);

};

} // end namespace world_model

} // end namespace ed

#endif
