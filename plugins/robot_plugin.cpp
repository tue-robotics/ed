#include "robot_plugin.h"

#include <kdl_parser/kdl_parser.hpp>

#include <ros/node_handle.h>

#include <ed/update_request.h>
#include <ed/world_model.h>
#include <ed/entity.h>

// URDF shape loading
#include <ros/package.h>
#include <geolib/Importer.h>
#include <geolib/Box.h>

#include <ed/world_model/transform_crawler.h>

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

geo::ShapePtr linkToShape(const boost::shared_ptr<urdf::Link>& link)
{
    geo::ShapePtr shape;

    if (!link->visual || !link->visual->geometry)
        return shape;

    geo::Pose3D offset;
    const urdf::Pose& o = link->visual->origin;
    offset.t = geo::Vector3(o.position.x, o.position.y, o.position.z);
    offset.R.setRotation(geo::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w));

    //            std::cout << link->name << ": " << offset << std::endl;
    //            std::cout << "    " << o.rotation.x << ", " << o.rotation.y<< ", " << o.rotation.z<< ", " << o.rotation.w << std::endl;

    if (link->visual->geometry->type == urdf::Geometry::MESH)
    {
        urdf::Mesh* mesh = static_cast<urdf::Mesh*>(link->visual->geometry.get());
        if (!mesh)
            return shape;

        std::string pkg_prefix = "package://";
        if (mesh->filename.substr(0, pkg_prefix.size()) == pkg_prefix)
        {
            std::string str = mesh->filename.substr(pkg_prefix.size());
            size_t i_slash = str.find("/");

            std::string pkg = str.substr(0, i_slash);
            std::string rel_filename = str.substr(i_slash + 1);
            std::string pkg_path = ros::package::getPath(pkg);
            std::string abs_filename = pkg_path + "/" + rel_filename;

            geo::Importer importer;
            shape = importer.readMeshFile(abs_filename, mesh->scale.x);

            if (!shape)
                std::cout << "RobotPlugin: Could not load shape" << std::endl;
        }
    }
    else if (link->visual->geometry->type == urdf::Geometry::BOX)
    {
        urdf::Box* box = static_cast<urdf::Box*>(link->visual->geometry.get());
        if (box)
        {
            double hx = box->dim.x / 2;
            double hy = box->dim.y / 2;
            double hz = box->dim.z / 2;

            shape.reset(new geo::Box(geo::Vector3(-hx, -hy, -hz), geo::Vector3(hx, hy, hz)));
        }
    }
    else if (link->visual->geometry->type == urdf::Geometry::CYLINDER)
    {
        urdf::Cylinder* cyl = static_cast<urdf::Cylinder*>(link->visual->geometry.get());
        if (!cyl)
            return shape;

        geo::Mesh mesh;

        int N = 20;

        // Calculate vertices
        for(int i = 0; i < N; ++i)
        {
            double a = 6.283 * i / N;
            double x = sin(a) * cyl->radius;
            double y = cos(a) * cyl->radius;

            mesh.addPoint(x, y, -cyl->length / 2);
            mesh.addPoint(x, y, cyl->length / 2);
        }

        // Calculate triangles
        for(int i = 1; i < N - 1; ++i)
        {
            int i2 = 2 * i;
            mesh.addTriangle(0, i2, i2 + 2);
            mesh.addTriangle(1, i2 + 1, i2 + 3);
        }

        for(int i = 0; i < N; ++i)
        {
            int j = (i + 1) % N;
            mesh.addTriangle(i * 2, j * 2, i * 2 + 1);
            mesh.addTriangle(i * 2 + 1, j * 2, j * 2 + 1);
        }

        shape.reset(new geo::Shape());
        shape->setMesh(mesh);
    }

    // Transform using visual offset
    shape->setMesh(shape->getMesh().getTransformed(offset));

    return shape;
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
    ed::UUID child_id = robot_name_ + "/" + segment.getName();

    // Set the entity type (robot_link)
    req.setType(child_id, "robot_link");

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

    if (!robot_model_.initString(urdf_xml))
    {
        std::cout << "Could not load robot model." << std::endl;
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
        std::vector<boost::shared_ptr<urdf::Link> > links;
        robot_model_.getLinks(links);

        for(std::vector<boost::shared_ptr<urdf::Link> >::const_iterator it = links.begin(); it != links.end(); ++it)
        {
            const boost::shared_ptr<urdf::Link>& link = *it;

            geo::ShapePtr shape = linkToShape(link);
            if (shape)
            {
                std::string id = robot_name_ + "/" + link->name;
                req.setShape(id, shape);
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
            if (e->shape())
            {
                req.setPose(e->id(), e_robot->pose() * tc.transform());
                req.setFlag(e->id(), "self"); // mark as self
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobotPlugin)
