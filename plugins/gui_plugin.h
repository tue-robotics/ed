#ifndef ED_GUI_PLUGIN_H_
#define ED_GUI_PLUGIN_H_

#include <ed/plugin.h>

#include <ed/types.h>

// Communication
#include <ros/callback_queue.h>
#include <ros/service_server.h>
#include <ros/publisher.h>

// Configuration
#include <tue/config/configuration.h>

// Services
#include <ed/GetMeasurements.h>
#include <ed/SetLabel.h>
#include <ed/RaiseEvent.h>
#include <ed/GetGUICommand.h>

// Map drawing
#include <geolib/sensors/DepthCamera.h>

#include <ed/event_clock.h>

class GUIPlugin : public ed::Plugin
{

public:

    GUIPlugin();

    virtual ~GUIPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

    ed::EventClock trigger_map_publish_;

    const ed::WorldModel* world_model_;


    // Map drawing

    geo::DepthCamera projector_;

    geo::Pose3D projector_pose_;

    cv::Mat map_image_;

    // Communication

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_get_measurements_;

    ros::ServiceServer srv_set_label_;

    ros::ServiceServer srv_raise_event_;

    ros::ServiceServer srv_get_command_;

    ros::Publisher pub_image_map_;

    bool srvGetMeasurements(ed::GetMeasurements::Request& req, ed::GetMeasurements::Response& res);

    bool srvSetLabel(ed::SetLabel::Request& req, ed::SetLabel::Response& res);

    bool srvRaiseEvent(ed::RaiseEvent::Request& req, ed::RaiseEvent::Response& res);

    bool srvGetCommand(ed::GetGUICommand::Request& req, ed::GetGUICommand::Response& res);


    void handleRequests();

    void publishMapImage();

    // Click

    cv::Point2i p_click_;

    ros::Time t_last_click_;

    // COMMAND

    std::string command_;

    std::string command_id_;

    std::map<std::string, std::string> command_params_;

    ros::Time t_command_;

    // SELECTION

    ed::UUID selected_id_;


    // HELPER FUNCTIONS

    ed::UUID getEntityFromClick(const cv::Point2i& p) const;

    cv::Point2i coordinateToPixel(const geo::Vector3& p) const;

    cv::Point2i coordinateToPixel(double x, double y, double z) const
    {
        return coordinateToPixel(geo::Vector3(x, y, z));
    }

    cv::Point2i coordinateToPixel(const pcl::PointXYZ& p) const
    {
        return coordinateToPixel(p.x, p.y, p.z);
    }

};

#endif
