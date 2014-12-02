#ifndef ED_TRANSFORM_CACHE_H_
#define ED_TRANSFORM_CACHE_H_

#include "ed/relation.h"
#include "ed/time_cache.h"

namespace ed
{

class TransformCache : public ed::Relation
{

public:

    TransformCache();

    ~TransformCache();

    bool calculateTransform(const Time& t, geo::Pose3D& tf) const;

    void insert(const Time& t, const geo::Pose3D& tf) { cache_.insert(t, tf); }

private:

    // Transforms, ordered in time
    TimeCache<geo::Pose3D> cache_;

};

} // end namespace ed

#endif
