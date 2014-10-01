#include "ed/perception_modules/perception_module.h"

#include "ed/measurement.h"

// Module loading
#include <ros/package.h>
#include <tue/filesystem/crawler.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

PerceptionModulePtr loadPerceptionModule(class_loader::ClassLoader* loader)
{
    loader->loadLibrary();
    std::vector<std::string> classes = loader->getAvailableClasses<ed::PerceptionModule>();

    if (classes.empty())
    {
        std::cout << "Error: could not find any perception modules in '" << loader->getLibraryPath() << "'." << std::endl;
        return PerceptionModulePtr();
    }

    if (classes.size() > 1)
    {
        std::cout << "Error: multiple perception modules registered in '" << loader->getLibraryPath() << "'." << std::endl;
        return PerceptionModulePtr();
    }

    PerceptionModulePtr perception_mod = loader->createInstance<PerceptionModule>(classes.front());

    std::string object_models_path = ros::package::getPath("ed_object_models");

    std::string config_path = object_models_path + "/configs/" + perception_mod->name();

    perception_mod->loadConfig(config_path);

    tue::filesystem::Crawler model_crawler(object_models_path + "/models");
    model_crawler.setRecursive(false);
    model_crawler.setListDirectories(true);
    model_crawler.setListFiles(false);

    tue::filesystem::Path model_path;
    while(model_crawler.nextPath(model_path))
    {
        std::string model_name = model_path.filename();
        tue::filesystem::Path full_model_path = model_path.join(perception_mod->name());

        if (full_model_path.isDirectory() && full_model_path.exists())
        {
            perception_mod->loadModel(model_name, full_model_path.string());
        }
    }

    return perception_mod;
}

}
