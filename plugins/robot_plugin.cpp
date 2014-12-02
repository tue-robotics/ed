#include "robot_plugin.h"

#include <kdl_parser/kdl_parser.hpp>

#include <ros/node_handle.h>

#include <ed/update_request.h>
#include <ed/world_model.h>
#include <ed/entity.h>

// ----------------------------------------------------------------------------------------------------

bool JointRelation::calculateTransform(const ed::Time& t, geo::Pose3D& tf) const
{
    ed::TimeCache<float>::const_iterator it_low, it_up;
    joint_pos_cache_.getLowerUpper(t, it_low, it_up);

    float joint_pos;

    if (it_low == joint_pos_cache_.end())
    {
        if (it_up == joint_pos_cache_.end())
            // No upper or lower bound (cache is empty)
            return false;

        // Requested time is in the past
        joint_pos = it_up->second;
    }
    else
    {
        if (it_up == joint_pos_cache_.end())
        {
            // Requested time is in the future
            joint_pos = it_low->second;
        }
        else
        {
            // Interpolate
            float p1 = it_low->second;
            float p2 = it_up->second;

            float dt1 = t.seconds() - it_low->first.seconds();
            float dt2 = it_up->first.seconds() - t.seconds();

            // Linearly interpolate joint positions
            joint_pos = (p1 * dt2 + p2 * dt1) / (dt1 + dt2);
        }
    }

    // Calculate joint pose for this joint position
    KDL::Frame pose_kdl = segment_.pose(joint_pos);

    // Convert to geolib transform
    tf.R = geo::Matrix3(pose_kdl.M.data);
    tf.t = geo::Vector3(pose_kdl.p.data);
}


// ----------------------------------------------------------------------------------------------------

RobotPlugin::RobotPlugin() : model_initialized_(true)
{
}

// ----------------------------------------------------------------------------------------------------

RobotPlugin::~RobotPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::constructRobot(const ed::UUID& parent_id, const KDL::SegmentMap::const_iterator& it_segment, ed::UpdateRequest& req)
{
    const KDL::Segment& segment = it_segment->second.segment;

    // Child ID is the segment (link) name
    ed::UUID child_id = segment.getName();

    // Set the entity type (robot_link)
    req.setType(child_id, "robot_link");

    // Create a joint relation and add id
    boost::shared_ptr<JointRelation> r(new JointRelation(segment));
    r->setCacheSize(joint_cache_size_);
    req.setRelation(parent_id, child_id, r);

    // Generate relation info that will be used to update the relation
    RelationInfo& rel_info = joint_name_to_rel_info_[segment.getJoint().getName()];
    rel_info.parent_id = parent_id;
    rel_info.child_id = child_id;
    rel_info.r_idx = ed::INVALID_IDX;
    rel_info.last_rel = r;


    // Recursively add all children
    const std::vector<KDL::SegmentMap::const_iterator>& children = it_segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++)
        constructRobot(child_id, children[i], req);
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->name.size() != msg->position.size())
    {
        std::cout << "[ED RobotPlugin] On joint callback: name and position vector must be of equal length." << std::endl;
        return;
    }

    for(unsigned int i = 0; i < msg->name.size(); ++i)
    {
        const std::string& name = msg->name[i];
        double pos = msg->position[i];

        std::map<std::string, RelationInfo>::iterator it_r = joint_name_to_rel_info_.find(name);
        if (it_r != joint_name_to_rel_info_.end())
        {
            RelationInfo& info = it_r->second;

            // Make a copy of the last relation
            boost::shared_ptr<JointRelation> r(new JointRelation(*info.last_rel));
            r->setCacheSize(joint_cache_size_);

            r->insert(msg->header.stamp.toSec(), pos);

            std::cout << name << ": " << r->size() << std::endl;

            update_req_->setRelation(info.parent_id, info.child_id, r);

            info.last_rel = r;
        }
        else
        {
            std::cout << "[ED RobotPlugin] On joint callback: unknown joint name '" << name << "'." << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::configure(tue::Configuration config)
{
    std::string urdf_rosparam;
    config.value("urdf_rosparam", urdf_rosparam);

    config.value("robot_name", robot_name_);

    ros::NodeHandle nh;

    if (config.readArray("joint_topics"))
    {
        while(config.nextArrayItem())
        {
            std::string topic;
            config.value("topic", topic);
            std::cout << "[RobotPlugin] Topic: " << topic << std::endl;

            ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<sensor_msgs::JointState>
                    (topic, 10, boost::bind(&RobotPlugin::jointCallback, this, _1), ros::VoidPtr(), &cb_queue_);

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

    if (!kdl_parser::treeFromString(urdf_xml, tree_))
    {
        config.addError("Could not initialize KDL tree object.");
        return;
    }

    joint_cache_size_ = 100; // TODO: remove magic number

    model_initialized_ = false;

}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void RobotPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    if (!model_initialized_)
    {
        constructRobot(robot_name_, tree_.getRootSegment(), req);
        model_initialized_ = true;
        return;
    }

    update_req_ = &req;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobotPlugin)
