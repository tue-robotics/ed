#ifndef ED_ROBOT_PLUGIN_H_
#define ED_ROBOT_PLUGIN_H_

#include <ed/plugin.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>

#include <kdl/tree.hpp>
#include <geolib/datatypes.h>

struct Joint
{
    std::string name;
    unsigned int parent_idx;
    unsigned int child_idx;
    double position;
    KDL::Segment segment; // calculates the joint pose
    geo::Pose3D transform;
};

struct Link
{
    std::string name;
};

class RobotPlugin : public ed::Plugin
{

public:

    RobotPlugin();

    virtual ~RobotPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    // Joint and link info

    std::map<std::string, Joint*> joints_;

    std::map<std::string, Link*> links_;


    // Transform graph

    Link* root_;

    std::vector<Link*> nodes_;

    std::vector<std::vector<Joint*> > edges_;

    void constructRobot(unsigned int parent_idx, const KDL::SegmentMap::const_iterator& it_segment);

    unsigned int addLink(Link* link);

    unsigned int addJoint(Joint* joint);


    // ROS Communication

    ros::CallbackQueue cb_queue_;

    std::map<std::string, ros::Subscriber> joint_subscribers_;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);


};

#endif
