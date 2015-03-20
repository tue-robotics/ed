#include "custom_properties_plugin.h"

#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/entity.h>

// Property info
#include "pose_info.h"
#include "counter_info.h"

// ----------------------------------------------------------------------------------------------------

CustomProperties::CustomProperties()
{
}

// ----------------------------------------------------------------------------------------------------

CustomProperties::~CustomProperties()
{
}

// ----------------------------------------------------------------------------------------------------

void CustomProperties::initialize(ed::InitData& init)
{
    // Register the entity property keys you want to use. The first argument is the name of the
    // property. This name is used to specify properties accross plugins. The second is the property
    // key, which will be initialized when calling this function

    init.properties.registerProperty("pose", k_pose_, new PoseInfo);
    init.properties.registerProperty("counter", k_counter_, new CounterInfo);
}

// ----------------------------------------------------------------------------------------------------

void CustomProperties::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    // Find the entity with id 'test-entity'
    ed::EntityConstPtr e = world.getEntity("test-entity");

    if (!e)
    {
        // Entity does not yet exist, so add it by setting the pose property. Note that
        // setting a property will automatically add the entity to the world model

        // To set a property you need to pass the id of the entity, the key of the property
        // you want to change, and the value of the property. In our case, we want to
        // initialize the 'pose' of the entity to the identity pose ((0, 0, 0) and zero rotation).
        req.setProperty("test-entity", k_pose_, geo::Pose3D::identity());
    }
    else
    {
        // Entity exists. Try to get his pose. Again, use the appropriate key to access the property
        const geo::Pose3D* pose = e->property(k_pose_);

        // The Entity::property function returns a const pointer. If this pointer is 0, the
        // property was not found OR the type did not match. Therefore, always check if the pointer is not null!
        if (pose)
        {
            std::cout << "Entity " << e->id() << " has pose " << *pose << std::endl;

            // Add a little offset to the current pose ...
            geo::Pose3D new_pose = *pose;
            new_pose.t += geo::Vector3(0.1, 0.2, 0.3);

            // .. and update it
            req.setProperty(e->id(), k_pose_, new_pose);
        }

        // Now try to get the counter property
        const int* counter = e->property(k_counter_);
        if (!counter)
        {
            // The first time the plugin tries to access the counter property, the property is not yet there.
            std::cout << "Counter property does not yet exist. Will be created and initialized to 0" << std::endl;
            req.setProperty(e->id(), k_counter_, 0);
        }
        else
        {
            // The second time the counter property will be set, so we can access it and update
            std::cout << "Entity " << e->id() << " has counter " << *counter << std::endl;
            req.setProperty(e->id(), k_counter_, *counter + 1);
        }

        std::cout << std::endl;
    }
}

ED_REGISTER_PLUGIN(CustomProperties)
