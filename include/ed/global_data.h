#ifndef ED_GLOBAL_DATA_H_
#define ED_GLOBAL_DATA_H_

// Data that is accessible from the world model and each entity

#include "ed/types.h"
#include "ed/property_key.h"
#include <geolib/datatypes.h>

namespace ed
{

class PropertyKeyDB;

struct GlobalData
{

    PropertyKey<geo::Pose3D> k_pose_;
    PropertyKey<std::string> k_type_;
    PropertyKey<geo::ShapeConstPtr> k_shape_;

    const PropertyKeyDB* property_key_db_;

};

} // end namespace ed

#endif
