#include "ed/perception.h"
#include "ed/entity.h"
#include "ed/measurement.h"

#include "ed/models/loader.h"

#include "ed/perception/aggregator.h"

#include <ros/package.h>

#include <ed/perception/model_fitter.h>

// Visualization
#include <opencv2/highgui/highgui.hpp>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

EntityPtr updateEntityType(const EntityConstPtr& e, const std::string& type, bool fit_shape)
{
    EntityPtr e_updated(new Entity(*e));

    // Update entity with label info
    e_updated->setType(type);

    if (fit_shape)
    {
        // Lookup the object shape in the object db
        ed::models::Loader loader;
        geo::ShapePtr shape = loader.loadShape(type);

        if (shape) {
            geo::Pose3D fitted_pose;
            if (ed::model_fitter::fit(*e, shape, fitted_pose))
            {
                // Set shape and pose
                e_updated->setShape(shape);
                e_updated->setPose(fitted_pose);
            }
        }
    }

    return e_updated;
}

// ----------------------------------------------------------------------------------------------------

std::string getBestLabel(const PerceptionResult& result)
{
    double score_threshold = 0.5; // TODO: get rid of hard-coded value

    double max_score = 0;
    const Percept* best_p = 0;

    const std::map<std::string, Percept>& percepts = result.percepts();
    for(std::map<std::string, Percept>::const_iterator it = percepts.begin(); it != percepts.end(); ++it)
    {
        const Percept& p = it->second;
        if (p.score > max_score && p.score > score_threshold)
        {
            max_score = p.score;
            best_p = &p;
        }
    }

    if (!best_p)
        return "";

    return best_p->label;
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

    // Make sure the perception module is destroyed by resetting all shared pointers
    perception_module_.reset();

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
    std::string ed_dir = ros::package::getPath("ed");
    std::string lib_dir = ed_dir + "/lib/";

    config.value("fit_shapes", fit_shape_);

    if (config.readArray("modules"))
    {
        boost::shared_ptr<ed::PerceptionAggregator> perception_aggregator(new PerceptionAggregator);

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
                    perception_aggregator->addPerceptionModule(perception_module);
                }
            }
        }

        config.endArray();

        perception_module_ = perception_aggregator;
    }
}

// ----------------------------------------------------------------------------------------------------

void Perception::update(std::map<UUID, EntityConstPtr>& entities)
{
    // Don't update if there is no perception module
    if (!perception_module_)
        return;

//    int num_visualization_images = 0;

    for(std::map<UUID, EntityConstPtr>::iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const UUID& id = it->first;
        const EntityConstPtr& e = it->second;

        if (e->type() != "")
        {
            std::map<ed::UUID, std::string>::iterator it2 = previous_entity_types_.find(e->id());
            if (it2 == previous_entity_types_.end() || it2->second != e->type())
            {
                // The entity received a type it did not have before. Therefore, update it and fit
                // a shape model
                it->second = updateEntityType(e, e->type(), fit_shape_);

                previous_entity_types_[id] = e->type();

                // We do not have to start a perception worker since we already have a type
                continue;
            }
        }

        std::map<UUID, PerceptionWorker*>::iterator it_worker = workers_.find(it->first);
        if (it_worker == workers_.end())
        {
            // No worker active for this entity, so create one

            // get the measurements from the entity
            std::vector<MeasurementConstPtr> measurements;
            e->measurements(measurements);

            // create worker and add measurements
            PerceptionWorker* worker = new PerceptionWorker();
            worker->setMeasurements(measurements);
            worker->setPerceptionModule(perception_module_);

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
                    worker->setMeasurements(measurements);
                    worker->start();
                }
            }
            // Check if it just finished processing
            else if (worker->isDone())
            {
                // Check if the worker has found something useful
                std::string label = getBestLabel(worker->getResult());
                if (!label.empty())
                {
                    // If so, assign the found label to the entity
                     it->second = updateEntityType(e, label, fit_shape_);
                }

//                // Get visualizations of perception module, and show if there are any
//                const std::map<std::string, cv::Mat>& vis_images = worker->getResult().visualizationImages();
//                if (!vis_images.empty())
//                {
//                    for(std::map<std::string, cv::Mat>::const_iterator it = vis_images.begin(); it != vis_images.end(); ++it)
//                    {
//                        cv::imshow(e->getID() + " " + it->first, it->second);
//                        ++num_visualization_images;
////                        cv::imshow("Test", it->second);
//                    }
//                }

                // Set worker to idle. This way, the result is not checked again on the next iteration
                worker->setIdle();

                worker->t_last_processing = worker->measurements.front()->timestamp();
            }
        }

    }

//    if (num_visualization_images > 0)
//        cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

}
