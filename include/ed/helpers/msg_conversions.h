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
 * @brief converting geo::ShapeConstPtr to ed_msgs::SubArea message
 * @param shape geo::ShapeConstPtr as input
 * @param msg filled ed_msgs::SubArea message as output
 */
void convert(const geo::ShapeConstPtr shape, ed_msgs::SubArea& sub_area)
{
    geo::Vector3 min = shape->getBoundingBox().getMin();
    geo::Vector3 max = shape->getBoundingBox().getMax();

    geo::Vector3 pos = (min + max)/2;
    geo::Vector3 size = max - min;

    geo::convert(pos, sub_area.center_point);

    shape_msgs::SolidPrimitive solid;
    sub_area.geometry.type = sub_area.geometry.BOX;
    sub_area.geometry.dimensions.resize(3, 0);
    sub_area.geometry.dimensions[solid.BOX_X] = size.x;
    sub_area.geometry.dimensions[solid.BOX_Y] = size.y;
    sub_area.geometry.dimensions[solid.BOX_Z] = size.z;
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
//    msg.creation_time = ros::Time(e.creationTime());

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

    if (!e.areas().empty())
    {
        msg.areas.resize(e.areas().size());
        int i=0;
        for (std::map<std::string, geo::ShapeConstPtr>::const_iterator it = e.areas().begin(); it != e.areas().end(); ++it)
        {
            ed_msgs::Area area;
            area.name = it->first;

            geo::CompositeShapeConstPtr composite = boost::dynamic_pointer_cast<const geo::CompositeShape>(it->second);
            if(composite)
            {
                std::vector<std::pair<geo::ShapePtr, geo::Transform> >  shapes = composite->getShapes();
                area.subareas.resize(shapes.size());
                int i2 = 0;
                for (std::vector<std::pair<geo::ShapePtr, geo::Transform> >::const_iterator it2 = shapes.begin();
                     it2 != shapes.end(); ++it2)
                {
                    geo::ShapePtr shape_tr(new geo::Shape());
                    shape_tr->setMesh(it2->first->getMesh().getTransformed(it2->second.inverse()));

                    ed_msgs::SubArea sub_area;
                    convert(shape_tr, sub_area);
                    area.subareas[i2] = sub_area;
                    ++i2;
                }
            }
            else
            {
                area.subareas.resize(1);
                ed_msgs::SubArea sub_area;

                convert(it->second, sub_area);

                area.subareas[0] = sub_area;
            }

            msg.areas[i] = area;
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
