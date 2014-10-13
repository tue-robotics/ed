#include "ed/perception.h"
#include "ed/entity.h"
#include "ed/measurement.h"

#include "ed/models/models.h"

//#include "ed/perception/aggregator.h"

#include <ros/package.h>

#include <ed/perception/model_fitter.h>

// Visualization
#include <opencv2/highgui/highgui.hpp>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

EntityPtr updateEntityType(const EntityConstPtr& e, tue::Configuration perception_result, bool fit_shape)
{
    EntityPtr e_updated(new Entity(*e));

    // TODO: tue::Configuration is NOT thread safe and NOT immutable. This may go wrong...
    tue::Configuration params;
    params.add(e->getConfig());
    params.add(perception_result);

    std::string type;
    if (params.value("type", type, tue::OPTIONAL))
        e_updated->setType(type);

    e_updated->setConfig(params);

    if (fit_shape)
    {
//        // Lookup the object shape in the object db
//        ed::models::Loader loader;
//        geo::ShapePtr shape = loader.loadShape(type);

//        if (shape) {
//            geo::Pose3D fitted_pose;
//            if (ed::model_fitter::fit(*e, shape, fitted_pose))
//            {
//                // Set shape and pose
//                e_updated->setShape(shape);
//                e_updated->setPose(fitted_pose);
//            }
//        }
    }

    return e_updated;
}

// ----------------------------------------------------------------------------------------------------

Perception::Perception()
{
}

// ----------------------------------------------------------------------------------------------------

Perception::~Perception()
{
    for(std::map<UUID, PerceptionWorker*>::iterator it = workers_.begin(); it != workers_.end(); ++it)
    {
        delete it->second;
    }

    // Make sure the perception modules are destroyed by resetting all shared pointers
    for(std::vector<PerceptionModuleConstPtr>::iterator it = perception_modules_.begin();
            it != perception_modules_.end(); ++it)
        it->reset();

    // Once the modules are destroyed, we can destroy the loaders
    for(std::vector<class_loader::ClassLoader*>::iterator it = perception_loaders_.begin(); it != perception_loaders_.end(); ++it)
    {
        delete *it;
    }
}

// ----------------------------------------------------------------------------------------------------

void Perception::configure(tue::Configuration config)
{
    // Get the ED directory
    std::string ed_dir;
    std::string lib_dir;

    if (getEnvironmentVariable("ED_PLUGIN_PATH", ed_dir))
    {
        std::stringstream ss(ed_dir);
        std::string item;
        while (std::getline(ss, item, ':')){
            ed_dir = item + "/";
            break;
        }
    }
    else
    {
        std::cout << "Error: Environment variable ED_PLUGIN_PATH not set. Getting the path through ros getPackage." << std::endl;
        ed_dir = ros::package::getPath("ed");
    }

    lib_dir = ed_dir;

    config.value("fit_shapes", fit_shape_);

    if (config.readArray("modules"))
    {
        while(config.nextArrayItem())
        {
            std::string lib;
            if (config.value("lib", lib))
            {
                std::string lib_file = lib_dir + lib;

                // Load the library
                class_loader::ClassLoader* class_loader = new class_loader::ClassLoader(lib_file);
                perception_loaders_.push_back(class_loader);

                // Create perception module
                PerceptionModulePtr perception_module = ed::loadPerceptionModule(class_loader);

                if (perception_module)
                {
                    // Configure the module if there is a 'parameters' group in the config
                    if (config.readGroup("parameters"))
                    {
                        perception_module->configure(config.limitScope());
                        config.endGroup();
                    }

                    // Add the perception module to the aggregator
                    perception_modules_.push_back(perception_module);
                }
            }
        }

        config.endArray();
    }
}

// ----------------------------------------------------------------------------------------------------

void Perception::update(std::map<UUID, EntityConstPtr>& entities)
{
    // Don't update if there are no perception modules
    if (perception_modules_.empty())
        return;

    for(std::map<UUID, EntityConstPtr>::iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const UUID& id = it->first;
        const EntityConstPtr& e = it->second;

//        if (e->type() != "")
//        {
//            std::map<ed::UUID, std::string>::iterator it2 = previous_entity_types_.find(e->id());
//            if (it2 == previous_entity_types_.end() || it2->second != e->type())
//            {
//                // The entity received a type it did not have before. Therefore, update it and fit
//                // a shape model
//                it->second = updateEntityType(e, e->type(), fit_shape_);

//                previous_entity_types_[id] = e->type();

//                // We do not have to start a perception worker since we already have a type
//                continue;
//            }
//        }

        std::map<UUID, PerceptionWorker*>::iterator it_worker = workers_.find(it->first);
        if (it_worker == workers_.end())
        {
            // No worker active for this entity, so create one

            // create worker and add measurements
            PerceptionWorker* worker = new PerceptionWorker();
            worker->setEntity(e);
            worker->setPerceptionModules(perception_modules_);

            workers_[id] = worker;
            worker->start();
        }
        else
        {
            // Already a worker active
            PerceptionWorker* worker = it_worker->second;

            // Check if it is idle, but has done work before
            if (worker->isIdle() && worker->t_last_processing > 0)
            {
                // Worker has already done work and finished. Check if we want to run it again

                // Get the latest measurements since the last measurement processed by the worker
                std::vector<MeasurementConstPtr> measurements;
                e->measurements(measurements, worker->t_last_processing);

                if (!measurements.empty())
                {
                    // There are new measurements, so run the worker again
                    worker->setEntity(e);
                    worker->start();
                }
            }
            // Check if it just finished processing
            else if (worker->isDone())
            {
                // Update the entity with the results from the worker
                it->second = updateEntityType(e, worker->getResult(), fit_shape_);

                // Set worker to idle. This way, the result is not checked again on the next iteration
                worker->setIdle();

                worker->t_last_processing = worker->timestamp();
            }
        }

    }
}

// ----------------------------------------------------------------------------------------------------

bool Perception::getEnvironmentVariable(const std::string& var, std::string& value)
{
     const char * val = ::getenv(var.c_str());
     if ( val == 0 )
         return false;

     value = val;
     return true;
}

// ----------------------------------------------------------------------------------------------------

}
