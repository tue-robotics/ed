#ifndef ED_ROBOT_PLUGIN_H_
#define ED_ROBOT_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/relation.h>
#include <ed/time_cache.h>
#include <ed/uuid.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <geolib/datatypes.h>
#include <kdl/tree.hpp>

#include <urdf/model.h>

#include <map>

// ----------------------------------------------------------------------------------------------------

class JointRelation : public ed::Relation
{

public:
    JointRelation(const KDL::Segment& segment) : segment_(segment) {}

    bool calculateTransform(const ed::Time& t, geo::Pose3D& tf) const override;

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
    ed::Idx r_idx{};
    boost::shared_ptr<const JointRelation> last_rel;
};

// ----------------------------------------------------------------------------------------------------

class RobotPlugin : public ed::Plugin
{

public:
    RobotPlugin();

    ~RobotPlugin() override;

    void configure(tue::Configuration config) override;

    void initialize() override;

    void process(const ed::WorldModel& world, ed::UpdateRequest& req) override;

private:
    std::string robot_name_;

    bool model_initialized_{true};

    KDL::Tree tree_;

    urdf::Model robot_model_;

    std::map<std::string, RelationInfo> joint_name_to_rel_info_;

    ed::UpdateRequest* update_req_{};

    unsigned int joint_cache_size_{};

    void constructRobot(const ed::UUID& parent_id,
                        const KDL::SegmentMap::const_iterator& it_segment,
                        ed::UpdateRequest& req);

    // ROS Communication

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_subscribers_;

    void jointCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);
};

#endif
