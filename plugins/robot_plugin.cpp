#include "robot_plugin.h"

#include <kdl_parser/kdl_parser.hpp>

#include <ros/node_handle.h>

#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/world_model.h>
#include <ed/models/shape_loader.h>

#include <geolib/CompositeShape.h>

// URDF shape loading
#include <ros/package.h>
#include <geolib/io/import.h>
#include <geolib/Box.h>

#include <ed/world_model/transform_crawler.h>

#include <tuple>

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

    return true;
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr URDFGeometryToShape(const urdf::GeometrySharedPtr& geom)
{
    geo::ShapePtr shape;

    if (geom->type == urdf::Geometry::MESH)
    {
        urdf::Mesh* mesh = static_cast<urdf::Mesh*>(geom.get());
        if (!mesh)
        {
            ROS_WARN_NAMED("RobotPlugin", "[RobotPlugin] Robot model error: No mesh geometry defined");
            return shape;
        }

        std::string pkg_prefix = "package://";
        if (mesh->filename.substr(0, pkg_prefix.size()) == pkg_prefix)
        {
            std::string str = mesh->filename.substr(pkg_prefix.size());
            size_t i_slash = str.find("/");

            std::string pkg = str.substr(0, i_slash);
            std::string rel_filename = str.substr(i_slash + 1);
            std::string pkg_path = ros::package::getPath(pkg);
            std::string abs_filename = pkg_path + "/" + rel_filename;

            shape = geo::io::readMeshFile(abs_filename, mesh->scale.x);

            if (!shape)
                ROS_ERROR_STREAM_NAMED("RobotPlugin", "[RobotPlugin] Could not load mesh shape from '" << abs_filename << "'");
        }
    }
    else if (geom->type == urdf::Geometry::BOX)
    {
        urdf::Box* box = static_cast<urdf::Box*>(geom.get());
        if (!box)
        {
            ROS_WARN_NAMED("RobotPlugin", "[RobotPlugin] Robot model error: No box geometry defined");
            return shape;
        }

        double hx = box->dim.x / 2;
        double hy = box->dim.y / 2;
        double hz = box->dim.z / 2;

        shape.reset(new geo::Box(geo::Vector3(-hx, -hy, -hz), geo::Vector3(hx, hy, hz)));
    }
    else if (geom->type == urdf::Geometry::CYLINDER)
    {
        urdf::Cylinder* cyl = static_cast<urdf::Cylinder*>(geom.get());
        if (!cyl)
        {
            ROS_WARN_NAMED("RobotPlugin", "[RobotPlugin] Robot model error: No cylinder geometry defined");
            return shape;
        }

        shape.reset(new geo::Shape());
        ed::models::createCylinder(*shape, cyl->radius, cyl->length, 20);
    }
    else if (geom->type ==  urdf::Geometry::SPHERE)
    {
        urdf::Sphere* sphere = static_cast<urdf::Sphere*>(geom.get());
        if (!sphere)
        {
            ROS_WARN_NAMED("RobotPlugin", "[RobotPlugin] Robot model error: No sphere geometry defined");
            return shape;
        }

        shape.reset(new geo::Shape());
        ed::models::createSphere(*shape, sphere->radius);
    }

    return shape;
}

// ----------------------------------------------------------------------------------------------------

std::tuple<geo::ShapePtr, geo::ShapePtr> LinkToShapes(const urdf::LinkSharedPtr& link)
{
    geo::CompositeShapePtr visual, collision;

    for (urdf::VisualSharedPtr& vis : link->visual_array)
    {
        const urdf::GeometrySharedPtr& geom = vis->geometry;
        if (!geom)
        {
            ROS_WARN_STREAM_NAMED("RobotPlugin" ,"[RobotPlugin] Robot model error: missing geometry for visual in link: '" << link->name << "'");
            continue;
        }

        geo::Pose3D offset;
        const urdf::Pose& o = vis->origin;
        offset.t = geo::Vector3(o.position.x, o.position.y, o.position.z);
        offset.R.setRotation(geo::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));

        geo::ShapePtr subshape = URDFGeometryToShape(geom);
        if (!subshape)
            continue;

        if (!visual)
            visual.reset(new geo::CompositeShape());
        visual->addShape(*subshape, offset);
    }

    for (urdf::CollisionSharedPtr& col : link->collision_array)
    {
        const urdf::GeometrySharedPtr& geom = col->geometry;
        if (!geom)
        {
            ROS_WARN_STREAM_NAMED("RobotPlugin" ,"[RobotPlugin] Robot model error: missing geometry for collision in link: '" << link->name << "'");
            continue;
        }

        geo::Pose3D offset;
        const urdf::Pose& o = col->origin;
        offset.t = geo::Vector3(o.position.x, o.position.y, o.position.z);
        offset.R.setRotation(geo::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));

        geo::ShapePtr subshape = URDFGeometryToShape(geom);
        if (!subshape)
            continue;

        if (!collision)
            collision.reset(new geo::CompositeShape());
        collision->addShape(*subshape, offset);
    }

    return {visual, collision};
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
    if (child_id.str().find(robot_name_) == std::string::npos)
        child_id = robot_name_ + "/" + segment.getName();

    // Set the entity type (robot_link) and flag (self)
    req.setType(child_id, "robot_link");
    req.setFlag(child_id, "self");

    // Create a joint relation and add id
    boost::shared_ptr<JointRelation> r(new JointRelation(segment));
    r->setCacheSize(joint_cache_size_);
    r->insert(0, 0);
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
        ROS_ERROR("[ED RobotPlugin] On joint callback: name and position vector must be of equal length.");
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

            update_req_->setRelation(info.parent_id, info.child_id, r);

            info.last_rel = r;
        }
        else
        {
            ROS_ERROR_STREAM("[ED RobotPlugin] On joint callback: unknown joint name '" << name << "'.");
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
            ROS_DEBUG_STREAM("[RobotPlugin] Topic: " << topic);

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

    if (!robot_model_.initString(urdf_xml))
    {
        config.addError("Could not load robot model.");
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
        // Create the links
        std::vector<urdf::LinkSharedPtr> links;
        robot_model_.getLinks(links);

        for(std::vector<urdf::LinkSharedPtr >::const_iterator it = links.begin(); it != links.end(); ++it)
        {
            const urdf::LinkSharedPtr& link = *it;

            geo::ShapePtr visual, collision;
            std::tie(visual, collision) = LinkToShapes(link);
            if (visual || collision)
            {
                std::string id = link->name;
                if (link->name.find(robot_name_) == std::string::npos)
                    id = robot_name_ + "/" + link->name;
                if (visual)
                    req.setVisual(id, visual);
                if (collision)
                    req.setCollision(id, collision);
            }
        }

        // Create the joints
        constructRobot(robot_name_, tree_.getRootSegment(), req);
        model_initialized_ = true;

        req.setType(robot_name_, "robot");

        return;
    }

    update_req_ = &req;
    cb_queue_.callAvailable();

    ed::EntityConstPtr e_robot = world.getEntity(robot_name_);
    if (e_robot && e_robot->has_pose())
    {
        // Calculate absolute poses
        for(ed::world_model::TransformCrawler tc(world, robot_name_, ros::Time::now().toSec()); tc.hasNext(); tc.next())
        {
            const ed::EntityConstPtr& e = tc.entity();
            req.setPose(e->id(), e_robot->pose() * tc.transform());
            req.setFlag(e->id(), "self"); // mark as self
        }
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobotPlugin)
