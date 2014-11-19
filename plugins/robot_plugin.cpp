#include "robot_plugin.h"

#include <kdl_parser/kdl_parser.hpp>

#include <ros/node_handle.h>


// ----------------------------------------------------------------------------------------------------

RobotPlugin::RobotPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

RobotPlugin::~RobotPlugin()
{
    for(std::map<std::string, Joint*>::iterator it = joints_.begin(); it != joints_.end(); ++it)
        delete it->second;

    for(std::map<std::string, Link*>::iterator it = links_.begin(); it != links_.end(); ++it)
        delete it->second;
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::constructRobot(unsigned int parent_idx, const KDL::SegmentMap::const_iterator& it_segment)
{
    const KDL::Segment& segment = it_segment->second.segment;

    // Create link
    Link* link(new Link);
    link->name = segment.getName();

    // Add link
    unsigned int child_idx = addLink(link);

    // Create joint
    Joint* joint(new Joint);
    joint->position = 0;
    joint->segment = segment;
    joint->parent_idx = parent_idx;
    joint->child_idx = child_idx;
    joint->name = segment.getJoint().getName();

    // Add joint
    addJoint(joint);

    // Calculate pose with default joint position (0)
    KDL::Frame pose_kdl = segment.pose(joint->position);
    joint->transform.R = geo::Matrix3(pose_kdl.M.data);
    joint->transform.t = geo::Vector3(pose_kdl.p.data);

    // Recursively add all children
    const std::vector<KDL::SegmentMap::const_iterator>& children = it_segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++)
        constructRobot(child_idx, children[i]);
}

// ----------------------------------------------------------------------------------------------------

unsigned int RobotPlugin::addLink(Link* link)
{
    unsigned int idx = nodes_.size();
    nodes_.push_back(link);
    links_[link->name] = link;
    return idx;
}

// ----------------------------------------------------------------------------------------------------

unsigned int RobotPlugin::addJoint(Joint* joint)
{
    joints_[joint->name] = joint;

    unsigned int idx = joint->parent_idx;
    if (idx >= edges_.size())
        edges_.resize(idx + 1);

    edges_[idx].push_back(joint);
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

    std::string urdf_xml;
    if (!nh.getParam(urdf_rosparam, urdf_xml))
    {
        config.addError("No such ROS parameter: '" + urdf_rosparam + "'.");
        return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree))
    {
        config.addError("Could not initialize KDL tree object.");
        return;
    }

    std::cout << "OK" << std::endl;

    // Add root link
    root_ = new Link;
    root_->name = "robot";
    unsigned int root_idx = addLink(root_);

    constructRobot(root_idx, tree.getRootSegment());
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
