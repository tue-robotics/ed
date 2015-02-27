#ifndef ed_occupancy_grid_publisher_plugin_h_
#define ed_occupancy_grid_publisher_plugin_h_

#include <ros/ros.h>
#include <geolib/datatypes.h>
#include <ed/plugin.h>
#include <ed/types.h>

#include <opencv2/opencv.hpp>

#include <ed/GetGoalArea.h>
#include <ros/callback_queue.h>

class OccupancyGridPublisherPlugin : public ed::Plugin
{

public:

    OccupancyGridPublisherPlugin() : specifier_(""), sim_time_(0) {}

    virtual ~OccupancyGridPublisherPlugin() {}

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    bool getMapData(const ed::WorldModel& world, std::vector<ed::EntityConstPtr>& entities_to_be_projected);

    void updateMap(const ed::EntityConstPtr& e, cv::Mat& map);

    void publishMapMsg (const cv::Mat& map);

    bool worldToMap (double wx, double wy, int& mx, int& my) const
    {
        if (wx < origin_.x || wy < origin_.y)
            return false;

        mx = (wx - origin_.x) / res_ ;
        my = (wy - origin_.y) / res_ ;

        if (mx < width_ && my < height_)
            return true;

        return false;
    }

    void mapToWorld (unsigned int mx, unsigned int my, double& wx, double& wy) const
    {
        wx = origin_.x + (mx + 0.5) * res_;
        wy = origin_.y + (my + 0.5) * res_;
    }

    ros::Publisher map_pub_;

    //! Configurarable parameters  --------------

    double frequency_;

    std::string frame_id_;
    std::string topic_;
    std::string specifier_;

    double sim_time_;

    geo::Vector3 origin_;
    int width_, height_;

    double res_;

    //! Get goal area service

    const ed::WorldModel* world_;

    bool srvGetGoalArea(const ed::GetGoalArea::Request& ros_req, ed::GetGoalArea::Response& ros_res);

    ros::ServiceServer srv_get_goal_area_;

    ros::CallbackQueue cb_queue_;

};

#endif
