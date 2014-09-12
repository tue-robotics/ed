#include "builder_plugin.h"

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/models/loader.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/ros/msg_conversions.h>

// ----------------------------------------------------------------------------------------------------

BuilderPlugin::BuilderPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

BuilderPlugin::~BuilderPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void BuilderPlugin::configure(tue::Configuration config)
{

}

// ----------------------------------------------------------------------------------------------------

void BuilderPlugin::initialize()
{
    ros::NodeHandle nh;
    ros::AdvertiseServiceOptions opt_set_entity =
            ros::AdvertiseServiceOptions::create<ed::SetEntity>(
                "/ed/set_entity", boost::bind(&BuilderPlugin::srvSetEntity, this, _1, _2), ros::VoidPtr(), &cb_queue_);
    srv_set_entity_ = nh.advertiseService(opt_set_entity);
}

// ----------------------------------------------------------------------------------------------------

void BuilderPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;
    update_req_ = &req;
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool BuilderPlugin::srvSetEntity(ed::SetEntity::Request& req, ed::SetEntity::Response& res)
{
    if (req.action == ed::SetEntity::Request::ADD)
    {
        ed::models::Loader l;
        geo::ShapePtr shape = l.loadShape(req.type);
        if (shape)
        {
            ed::EntityPtr e(new ed::Entity(req.id, req.type));
            e->setShape(shape);

            // Set the pose
            geo::Pose3D pose;
            geo::convert(req.pose, pose);
            e->setPose(pose);

            update_req_->setEntity(e);
        }
        else
        {
            res.error_msg = "No shape could be loaded for type '" + req.type + "'.";
        }
    }
    else if (req.action == ed::SetEntity::Request::DELETE)
    {
        update_req_->removeEntity(req.id);
    }
    else if (req.action == ed::SetEntity::Request::UPDATE_POSE)
    {
        ed::EntityConstPtr e = world_model_->getEntity(req.id);
        if (e)
        {
            geo::Pose3D new_pose;
            geo::convert(req.pose, new_pose);

            ed::EntityPtr e_new(new ed::Entity(*e));
            e_new->setPose(new_pose);

            update_req_->setEntity(e_new);
        }
        else
        {
            res.error_msg = "Entity '" + req.id + "' does not exist.";
        }
    }

    return true;
}

ED_REGISTER_PLUGIN(BuilderPlugin)
