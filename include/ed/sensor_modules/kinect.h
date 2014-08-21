#ifndef kinect_h_
#define kinect_h_

#include "ed/sensor_modules/sensor_module.h"

#include <rgbd/Client.h>
#include <tue/profiling/profiler.h>
#include <tue/profiling/ros/profile_publisher.h>

namespace ed
{

class Kinect : public SensorModule
{

public:

    Kinect(const tf::TransformListener& tf_listener);

    void configure(tue::Configuration config, bool reconfigure = false);

    void update(std::map<UUID, EntityConstPtr>& entities);

private:
    //! Profiling
    tue::ProfilePublisher pub_profile_;
    tue::Profiler profiler_;

    //! Sensor data
    rgbd::Client rgbd_client_;

    //! Processing
    std::map<std::string, RGBDALModulePtr> al_modules_;
    std::map<std::string, RGBDSegModulePtr> seg_modules_;

};

}

#endif
