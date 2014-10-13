#include "type_aggregator.h"


#include "ed/measurement.h"
#include <ed/entity.h>

// ----------------------------------------------------------------------------------------------------

TypeAggregator::TypeAggregator():
    PerceptionModule("type_aggregator"),
    init_success_(false)
{
    std::cout << "TypeAggregator class created!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

TypeAggregator::~TypeAggregator()
{
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::loadModel(const std::string& model_name, const std::string& model_path)
{
    /*
    Load any model specific data here
        model_name: the name of the model (e.g. 'human')
        model_path: the directory in which the models are stored
    */
    if (model_name.compare("aggregator") == 0){

        kModuleName = "type_aggregator";

        std::cout << "[" << kModuleName << "] " << "Finished loading" << std::endl;

        init_success_ = true;
    }
}

// ----------------------------------------------------------------------------------------------------

void TypeAggregator::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    if (result.readArray("plugins"))
    {
       std::cout << "[" << kModuleName << "] " << "found perception_result!" << std::endl;

        while(result.nextArrayItem())
        {
            std::cout << "[" << kModuleName << "] " << "next array item" << std::endl;
            std::string name;
            if (result.value("name", name))
            {
                std::cout << "[" << kModuleName << "] " << "name: " << name << std::endl;

//                std::map<std::string, SensorModulePtr>::iterator it_sensor = sensors_.find(name);

//                if (it_sensor == sensors_.end())
//                {
//                    // Sensor module does not yet exist. Determine the type and create a sensor
//                    // module accordingly.

//                    std::string type;
//                    if (config.value("type", type))
//                    {
//                        if (type == "kinect")
//                        {
//                            SensorModulePtr sensor_mod(new Kinect(tf_listener_));
//                            sensor_mod->configure(config);
//                            sensors_[name] = sensor_mod;
//                        }
//                    }
                }
//                else
//                {
//                    // Sensor module exists, so reconfigure
//                    it_sensor->second->configure(config, true);
//                }
//            }
        }

//        config.endArray();
    }
    else{
//        std::cout << "[" << kModuleName << "] " << "no array found in perception_result" << std::endl;
    }

}

ED_REGISTER_PERCEPTION_MODULE(TypeAggregator)
