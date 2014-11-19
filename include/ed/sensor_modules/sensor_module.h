#ifndef sensor_module_h_
#define sensor_module_h_

#include "ed/types.h"

#include <tf/transform_listener.h>
#include <geolib/ros/tf_conversions.h>

#include <tue/config/configuration.h>
#include <visualization_msgs/Marker.h>

namespace ed
{

class SensorModule
{

public:
    SensorModule(const tf::TransformListener& tf_listener, TYPE type) : tf_listener_(tf_listener), type_(type)
    {
        ros::NodeHandle nh("~/" + type_);
        vis_marker_pub_ = nh.advertise<visualization_msgs::Marker>("vis_markers",0);
    }
    virtual ~SensorModule() {}

    virtual void configure(tue::Configuration config, bool reconfigure = false) = 0;

    virtual void update(const WorldModelConstPtr& world_model, UpdateRequest& req) = 0;

protected:
    UUID source_, frame_;

    bool getSensorPoseMap(const double time_stamp, geo::Pose3D& sensor_pose)
    {
        try {
            tf::StampedTransform t_sensor_pose;
            tf_listener_.lookupTransform("map", frame_, ros::Time(time_stamp), t_sensor_pose);
            geo::convert(t_sensor_pose, sensor_pose);
        } catch(tf::TransformException& ex) {
            return false;
        }
        return true;
    }

    const tf::TransformListener& tf_listener_;

    TYPE type_;
    ros::Publisher vis_marker_pub_;
    bool visualize_;

};

}

#endif
