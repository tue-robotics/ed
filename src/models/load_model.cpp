#include <iostream>

// ROS
#include <ros/console.h>

// TU/e Robotics
#include <tue/filesystem/path.h>
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
        tue::filesystem::Path path(source);
        if (!path.exists())
        {
            ROS_ERROR_STREAM("Couldn't open: '" << path << "', because it doesn't exist");
            return false;
        }

        tue::config::ReaderWriter config;
        std::string extension = tue::filesystem::Path(source).extension();
        if ( extension == ".sdf" || extension == ".world")
            tue::config::loadFromSDFFile(source, config);
        else if (extension == ".xml")
            tue::config::loadFromXMLFile(source, config);
        else if (extension == ".yml" || extension == ".yaml")
            tue::config::loadFromYAMLFile(source, config);
        else
        {
            ROS_ERROR_STREAM("[model_viewer] extension: '" << extension << "'  is not supported.");
            return false;
        }

        if (!model_loader.create(config.data(), req, error))
        {
            ROS_ERROR_STREAM("File '" << source << "' could not be loaded:" <<
                             "\nError:\n" << error.str());
            return false;
        }
    }
    else if (load_type == LoadType::MODEL)
    {
        if (!model_loader.create("_root", source, req, error, true))
        {
            ROS_ERROR_STREAM("Model '" << source << "' could not be loaded:" <<
                             "\nError:\n" << error.str());
            return false;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Unknown load type");
        return false;
    }

    return true;

}

}  // End of namespace 'models'

}  // End of namespace 'ed'
