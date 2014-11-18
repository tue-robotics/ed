#ifndef ED_ROBOT_PLUGIN_H_
#define ED_ROBOT_PLUGIN_H_

#include <ed/plugin.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>

class RobotPlugin : public ed::Plugin
{

public:

    RobotPlugin();

    virtual ~RobotPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    ros::CallbackQueue cb_queue_;

    std::map<std::string, ros::Subscriber> joint_subscribers_;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

};

#endif
