#ifndef ED_ROBOT_PLUGIN_H_
#define ED_ROBOT_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/relation.h>
#include <ed/time_cache.h>
#include <ed/uuid.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/JointState.h>

#include <kdl/tree.hpp>
#include <geolib/datatypes.h>

#include <urdf/model.h>

// ----------------------------------------------------------------------------------------------------

class JointRelation : public ed::Relation
{

public:

    JointRelation(const KDL::Segment& segment) : segment_(segment) {}

    bool calculateTransform(const ed::Time& t, geo::Pose3D& tf) const;

    void insert(const ed::Time& t, float joint_pos) { joint_pos_cache_.insert(t, joint_pos); }

    inline unsigned int size() const { return joint_pos_cache_.size(); }

    void setCacheSize(unsigned int n) { joint_pos_cache_.setMaxSize(n); }

private:

    ed::TimeCache<float> joint_pos_cache_;
    KDL::Segment segment_; // calculates the joint pose

};


// ----------------------------------------------------------------------------------------------------

struct RelationInfo
{
    ed::UUID parent_id;
    ed::UUID child_id;
    ed::Idx r_idx;
    boost::shared_ptr<const JointRelation> last_rel;
};

// ----------------------------------------------------------------------------------------------------

class RobotPlugin : public ed::Plugin
{

public:

    RobotPlugin();

    virtual ~RobotPlugin();

    void configure(tue::Configuration config);

    void initialize();

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::string robot_name_;

    bool model_initialized_;

    KDL::Tree tree_;

    urdf::Model robot_model_;

    std::map<std::string, RelationInfo> joint_name_to_rel_info_;

    ed::UpdateRequest* update_req_;

    unsigned int joint_cache_size_;

    void constructRobot(const ed::UUID& parent_id, const KDL::SegmentMap::const_iterator& it_segment, ed::UpdateRequest& req);


    // ROS Communication

    ros::CallbackQueue cb_queue_;

    std::map<std::string, ros::Subscriber> joint_subscribers_;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);


};

#endif
