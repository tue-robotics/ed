#ifndef rgbd_seg_module_h_
#define rgbd_seg_module_h_

#include "ed/types.h"
#include "ed/rgbd_data.h"
#include <tue/config/configuration.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <visualization_msgs/Marker.h>

namespace ed
{

class RGBDSegModule
{

public:

    RGBDSegModule(const TYPE& type) : type_(type)
    {
        ros::NodeHandle nh("~/" + type_);
        vis_marker_pub_ = nh.advertise<visualization_msgs::Marker>("vis_markers",0);
    }

    virtual void process(const RGBDData& rgbd_data, std::vector<PointCloudMaskPtr>& segments) = 0;

    virtual void configure(tue::Configuration config) = 0;

protected:

    TYPE type_;

    ros::Publisher vis_marker_pub_;
    bool visualize_;

};

}

#endif
