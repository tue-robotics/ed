#ifndef ED_HELPERS_MSG_CONVERSIONS_H_
#define ED_HELPERS_MSG_CONVERSIONS_H_

#include "ed_msgs/EntityInfo.h"
#include "ed/entity.h"

#include "geolib/Shape.h"
#include "geolib/Box.h"
#include "geolib/CompositeShape.h"
#include "geolib/datatypes.h"
#include "geolib/ros/msg_conversions.h"

#include "tue/config/yaml_emitter.h"

#include <shape_msgs/SolidPrimitive.h>

namespace ed {

// ------------------------------ TO ROS ------------------------------

/**
 * @brief converting geo::ShapeConstPtr to ed_msgs::SubVolume message
 * @param shape geo::ShapeConstPtr as input
 * @param msg filled ed_msgs::SubVolume message as output
 */
void convert(const geo::ShapeConstPtr shape, ed_msgs::SubVolume& sub_Volume)
{
    geo::Vector3 min = shape->getBoundingBox().getMin();
    geo::Vector3 max = shape->getBoundingBox().getMax();

    geo::Vector3 pos = (min + max)/2;
    geo::Vector3 size = max - min;

    geo::convert(pos, sub_Volume.center_point);

    shape_msgs::SolidPrimitive solid;
    sub_Volume.geometry.type = sub_Volume.geometry.BOX;
    sub_Volume.geometry.dimensions.resize(3, 0);
    sub_Volume.geometry.dimensions[solid.BOX_X] = size.x;
    sub_Volume.geometry.dimensions[solid.BOX_Y] = size.y;
    sub_Volume.geometry.dimensions[solid.BOX_Z] = size.z;
}

/**
 * @brief converting ed::Entity to ed_msgs::EntityInfo message
 * @param e ed::Entity as input
 * @param msg filled ed_msgs::EntityInfo message as output
 */
void convert(const ed::Entity& e, ed_msgs::EntityInfo& msg) {
    msg.id = e.id().str();
    msg.type = e.type();

    for(std::set<std::string>::const_iterator it = e.types().begin(); it != e.types().end(); ++it)
        msg.types.push_back(*it);

    msg.existence_probability = e.existenceProbability();

    // Convex hull
    const ed::ConvexHull& convex_hull = e.convexHull();
    if (!convex_hull.points.empty())
    {
        msg.z_min = convex_hull.z_min;
        msg.z_max = convex_hull.z_max;

        msg.convex_hull.resize(convex_hull.points.size());
        for(unsigned int i = 0; i < msg.convex_hull.size(); ++i)
        {
            msg.convex_hull[i].x = convex_hull.points[i].x;
            msg.convex_hull[i].y = convex_hull.points[i].y;
            msg.convex_hull[i].z = 0;
        }
    }

    msg.has_shape = e.shape() ? true : false;
    msg.has_pose = e.has_pose();
    if (e.has_pose())
        geo::convert(e.pose(), msg.pose);

    msg.last_update_time =  ros::Time(e.lastUpdateTimestamp());

    if (!e.data().empty())
    {
        std::stringstream ss;
        tue::config::YAMLEmitter emitter;
        emitter.emit(e.data(), ss);

        msg.data = ss.str();
    }

    if (!e.volumes().empty())
    {
        msg.volumes.resize(e.volumes().size());
        int i=0;
        for (std::map<std::string, geo::ShapeConstPtr>::const_iterator it = e.volumes().begin(); it != e.volumes().end(); ++it)
        {
            ed_msgs::Volume volume;
            volume.name = it->first;

            geo::CompositeShapeConstPtr composite = boost::dynamic_pointer_cast<const geo::CompositeShape>(it->second);
            if (composite)
            {
                std::vector<std::pair<geo::ShapePtr, geo::Transform> >  shapes = composite->getShapes();
                volume.subvolumes.resize(shapes.size());
                int i2 = 0;
                for (std::vector<std::pair<geo::ShapePtr, geo::Transform> >::const_iterator it2 = shapes.begin();
                     it2 != shapes.end(); ++it2)
                {
                    geo::ShapePtr shape_tr(new geo::Shape());
                    shape_tr->setMesh(it2->first->getMesh().getTransformed(it2->second.inverse()));

                    ed_msgs::SubVolume sub_volume;
                    convert(shape_tr,  volume.subvolumes[i2]);
                    ++i2;
                }
            }
            else
            {
                volume.subvolumes.resize(1);
                ed_msgs::SubVolume sub_volume;

                convert(it->second, sub_volume);

                volume.subvolumes[0] = sub_volume;
            }

            msg.volumes[i] = volume;
            ++i;
        }
    }

    // Flags
    for(std::set<std::string>::const_iterator it = e.flags().begin(); it != e.flags().end(); ++it)
        msg.flags.push_back(*it);
}

// ------------------------------ FROM ROS ------------------------------

}

#endif
