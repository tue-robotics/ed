#include "ed/perception.h"
#include "ed/entity.h"
#include "ed/measurement.h"

#include "ed/models/models.h"

//#include "ed/perception/aggregator.h"

#include <ros/package.h>

#include <ed/perception/model_fitter.h>

#include <tue/filesystem/path.h>

// Visualization
#include <opencv2/highgui/highgui.hpp>

#include <tue/config/reader.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

EntityPtr updateEntityType(const EntityConstPtr& e, const tue::config::DataConstPointer& perception_result)
{
    EntityPtr e_updated(new Entity(*e));

    tue::config::DataPointer params;
    params.add(e->data());
    params.add(perception_result);

    tue::config::Reader r(params);
    std::string type;
    if (r.value("type", type, tue::config::OPTIONAL))
        e_updated->setType(type);

    e_updated->setData(params);

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
    // Get the plugin paths
    std::string ed_plugin_paths;
    if (getEnvironmentVariable("ED_PLUGIN_PATH", ed_plugin_paths))
    {
        std::stringstream ss(ed_plugin_paths);
        std::string item;
        while (std::getline(ss, item, ':'))
            plugin_paths_.push_back(item);
    }
    else
    {
        config.addError("Environment variable ED_PLUGIN_PATH not set.");
        return;
    }

    if (config.readArray("modules"))
    {
        while(config.nextArrayItem())
        {
            std::string lib;
            if (config.value("lib", lib))
            {
                std::string lib_file;
                for(std::vector<std::string>::const_iterator it = plugin_paths_.begin(); it != plugin_paths_.end(); ++it)
                {
                    std::string lib_file_test = *it + "/" + lib;
                    if (tue::filesystem::Path(lib_file_test).exists())
                    {
                        lib_file = lib_file_test;
                        break;
                    }
                }

                if (lib_file.empty())
                {
                    config.addError("Perception plugin '" + lib + "' could not be found.");
                    return;
                }

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
                if (worker->getResult().valid())
                    it->second = updateEntityType(e, worker->getResult());

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
