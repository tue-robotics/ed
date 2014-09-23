#ifndef ed_occupancy_grid_publisher_plugin_h_
#define ed_occupancy_grid_publisher_plugin_h_

#include <ros/ros.h>
#include <geolib/datatypes.h>
#include <ed/plugin.h>
#include <ed/types.h>

#include <opencv2/opencv.hpp>

class OccupancyGridPublisherPlugin : public ed::Plugin
{

public:

    OccupancyGridPublisherPlugin() : specifier_("") {}

    virtual ~OccupancyGridPublisherPlugin() {}

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    void publishMapMsg (const cv::Mat& map);

    void updateMap(const ed::EntityConstPtr& e, cv::Mat& map);

    bool worldToMap (double wx, double wy, int& mx, int& my) const;

    void mapToWorld (unsigned int mx, unsigned int my, double& wx, double& wy) const;

    ros::Publisher map_pub_;

    //! Configurarable parameters  --------------

    double frequency_;

    std::string frame_id_;
    std::string topic_;
    std::string specifier_;

    geo::Vector3 origin_;
    int width_, height_;

    double res_;

};

#endif
