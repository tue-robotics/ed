#include <filesystem>

// ED
#include "ed/logging.h"

// TU/e Robotics
#include <sstream>
#include <string>
#include <tue/config/loaders/sdf.h>
#include <tue/config/loaders/xml.h>
#include <tue/config/loaders/yaml.h>
#include <tue/config/reader_writer.h>

// ED
#include "ed/models/model_loader.h"
#include "ed/update_request.h"

namespace ed::models
{

bool loadModel(const enum LoadType load_type, const std::string& source, ed::UpdateRequest& req)
{
    ed::models::ModelLoader model_loader;
    std::stringstream error;
    if (load_type == LoadType::FILE)
    {
        std::filesystem::path const path(source);
        if (!std::filesystem::exists(path))
        {
            ed::log::error() << "Couldn't open: '" << source << "', because it doesn't exist" << '\n';
            return false;
        }

        tue::config::ReaderWriter config;
        std::string const extension = std::filesystem::path(source).extension().string();
        if (extension == ".sdf" || extension == ".world")
            tue::config::loadFromSDFFile(source, config);
        else if (extension == ".xml")
            tue::config::loadFromXMLFile(source, config);
        else if (extension == ".yml" || extension == ".yaml")
            tue::config::loadFromYAMLFile(source, config);
        else
        {
            ed::log::error() << "[model_viewer] extension: '" << extension << "'  is not supported." << '\n';
            return false;
        }

        if (!model_loader.create(config.data(), req, error))
        {
            ed::log::error() << "File '" << source << "' could not be loaded:" << "\nError:\n" << error.str() << '\n';
            return false;
        }
    }
    else if (load_type == LoadType::MODEL)
    {
        if (!model_loader.create("_root", source, req, error, true))
        {
            ed::log::error() << "Model '" << source << "' could not be loaded:" << "\nError:\n" << error.str() << '\n';
            return false;
        }
    }
    else
    {
        ed::log::error() << "Unknown load type" << '\n';
        return false;
    }

    return true;
}

} // namespace ed::models
