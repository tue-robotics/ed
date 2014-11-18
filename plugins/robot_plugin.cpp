#include "robot_plugin.h"

#include <ros/node_handle.h>

// ----------------------------------------------------------------------------------------------------

RobotPlugin::RobotPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

RobotPlugin::~RobotPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::cout << *msg << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::configure(tue::Configuration config)
{
    std::string urdf_rosparam;
    config.value("urdf_rosparam", urdf_rosparam);

    std::string robot_name;
    config.value("robot_name", robot_name);

    ros::NodeHandle nh;

    if (config.readArray("joint_topics"))
    {
        while(config.nextArrayItem())
        {
            std::string topic;
            config.value("topic", topic);
            std::cout << "[RobotPlugin] Topic: " << topic << std::endl;

            ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<sensor_msgs::JointState>
                    (topic, 1, boost::bind(&RobotPlugin::jointCallback, this, _1), ros::VoidPtr(), &cb_queue_);

            joint_subscribers_[topic] = nh.subscribe(sub_options);
        }

        config.endArray();
    }

    if (config.hasError())
        return;

}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobotPlugin)
