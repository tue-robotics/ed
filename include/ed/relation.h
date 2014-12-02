#ifndef ED_RELATION_H_
#define ED_RELATION_H_

#include "ed/types.h"
#include "ed/time.h"

#include <geolib/datatypes.h>

namespace ed
{

class Relation
{

public:

    virtual bool calculateTransform(const Time& t, geo::Pose3D& tf) const { return false; }

private:

    Idx parent_idx_;
    Idx child_idx_;

};

} // end namespace ed

#endif
