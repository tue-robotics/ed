#include "ed/perception_modules/perception_module.h"

#include "ed/measurement.h"

// Module loading
#include <ros/package.h>
#include <tue/filesystem/crawler.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

PerceptionModulePtr loadPerceptionModule(class_loader::ClassLoader* loader, const std::string& model_list_name)
{
    std::string module_name_ = "perception_module";

    loader->loadLibrary();
    std::vector<std::string> classes = loader->getAvailableClasses<ed::PerceptionModule>();

    if (classes.empty())
    {
        std::cout << "[" << module_name_ << "] " <<  "Error: could not find any perception modules in '" << loader->getLibraryPath() << "'." << std::endl;
        return PerceptionModulePtr();
    }

    if (classes.size() > 1)
    {
        std::cout << "[" << module_name_ << "] " <<  "Error: multiple perception modules registered in '" << loader->getLibraryPath() << "'." << std::endl;
        return PerceptionModulePtr();
    }

    std::vector<std::string> model_list;

    PerceptionModulePtr perception_mod = loader->createInstance<PerceptionModule>(classes.front());

    std::cout << "[" << module_name_ << "] " << "Loading perception module: " << perception_mod->name() << std::endl;

    std::string object_models_path = ros::package::getPath("ed_object_models");

    std::string config_path = object_models_path + "/configs/" + perception_mod->name();

    std::string model_list_path = object_models_path + "/configs/model_lists/" + model_list_name;

    perception_mod->loadConfig(config_path);

    if(!model_list_name.empty()){
        if(!loadModelList(model_list_path, model_list))
            std::cout << "[" << module_name_ << "] " << "Could not find model list at " << model_list_path << std::endl;
    }else{
        std::cout << "[" << module_name_ << "] " << "No model list specified, loading all models" << std::endl;
    }

    tue::filesystem::Crawler model_crawler(object_models_path + "/models");
    model_crawler.setRecursive(false);
    model_crawler.setListDirectories(true);
    model_crawler.setListFiles(false);

    tue::filesystem::Path model_path;
    while(model_crawler.nextPath(model_path))
    {
        std::string model_name = model_path.filename();
        tue::filesystem::Path full_model_path = model_path ; //.join(perception_mod->name());
        bool in_model_list = (std::find(model_list.begin(), model_list.end(), model_name) != model_list.end() || model_list.empty());

        if (full_model_path.isDirectory() && full_model_path.exists() && in_model_list)
        {
            perception_mod->loadModel(model_name, full_model_path.string());
        }
    }

    return perception_mod;
}

// ----------------------------------------------------------------------------------------------------

bool loadModelList(std::string& model_list_path, std::vector<std::string>& model_list){
    tue::Configuration conf;
    std::string model_name;

    if (conf.loadFromYAMLFile(model_list_path)){    // read YAML configuration
        if (conf.readArray("models")){              // read Model group

            while(conf.nextArrayItem()){
                if(conf.value("name", model_name))
                    model_list.push_back(model_name);
            }

            conf.endArray();    // close Models group
        }else{
            std::cout << "[" << "perception_module" << "] " << "Could not find 'models' group" << std::endl;
            return false;
        }
    }else{
//        std::cout << "[" << "perception_module" << "] " << "Could not load YML file." << std::endl;
        return false;
    }

    return true;
}

}
