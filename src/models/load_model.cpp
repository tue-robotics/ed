#include <filesystem>
#include <iostream>

// ED
#include "ed/logging.h"

// TU/e Robotics
#include <tue/config/configuration.h>
#include <tue/config/loaders/sdf.h>
#include <tue/config/loaders/xml.h>
#include <tue/config/loaders/yaml.h>

// ED
#include "ed/update_request.h"
#include "ed/models/model_loader.h"

namespace ed {

namespace models {

bool loadModel(const enum LoadType load_type, const std::string& source, ed::UpdateRequest& req)
{
    ed::models::ModelLoader model_loader;
    std::stringstream error;
    if (load_type == LoadType::FILE)
    {
        std::filesystem::path path(source);
        if (!std::filesystem::exists(path))
        {
            ed::log::error() << "Couldn't open: '" << source << "', because it doesn't exist" << std::endl;
            return false;
        }

        tue::config::ReaderWriter config;
        std::string extension = std::filesystem::path(source).extension().string();
        if ( extension == ".sdf" || extension == ".world")
            tue::config::loadFromSDFFile(source, config);
        else if (extension == ".xml")
            tue::config::loadFromXMLFile(source, config);
        else if (extension == ".yml" || extension == ".yaml")
            tue::config::loadFromYAMLFile(source, config);
        else
        {
            ed::log::error() << "[model_viewer] extension: '" << extension << "'  is not supported." << std::endl;
            return false;
        }

        if (!model_loader.create(config.data(), req, error))
        {
            ed::log::error() << "File '" << source << "' could not be loaded:" <<
                             "\nError:\n" << error.str() << std::endl;
            return false;
        }
    }
    else if (load_type == LoadType::MODEL)
    {
        if (!model_loader.create("_root", source, req, error, true))
        {
            ed::log::error() << "Model '" << source << "' could not be loaded:" <<
                             "\nError:\n" << error.str() << std::endl;
            return false;
        }
    }
    else
    {
        ed::log::error() << "Unknown load type" << std::endl;
        return false;
    }

    return true;

}

}  // End of namespace 'models'

}  // End of namespace 'ed'
