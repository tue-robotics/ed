#include "ed/models/model_loader.h"

#include "ed/update_request.h"
#include "ed/entity.h"
#include "ed/relations/transform_cache.h"

#include <tue/filesystem/path.h>

#include "shape_loader.h"

#include <tue/config/reader.h>
#include <tue/config/writer.h>
#include <tue/config/configuration.h>

#include <geolib/CompositeShape.h>

#include <sstream>

namespace ed
{

namespace models
{

bool readSDFGeometry(const std::string& model_path, tue::config::Reader r, geo::CompositeShapePtr& composite, std::stringstream& error, geo::Pose3D pose_offset = geo::Pose3D::identity())
{
    geo::Pose3D pose = geo::Pose3D::identity();
    readPose(r, pose);
    pose = pose_offset * pose;
    if (r.readGroup("geometry"))
    {
        std::map<std::string, geo::ShapePtr> dummy_shape_cache;
        geo::ShapePtr sub_shape = loadShape(model_path, r, dummy_shape_cache, error);
        if (sub_shape)
        {
            if (!composite)
                composite.reset(new geo::CompositeShape);
            composite->addShape(*sub_shape, pose);
        }
        r.endGroup();
        return true;
    }
    else
        return false;
}

// ----------------------------------------------------------------------------------------------------

ModelLoader::ModelLoader()
{
    const char * mpath = ::getenv("ED_MODEL_PATH");
    if (mpath)
    {
        std::stringstream ss(mpath);
        std::string item;
        while (std::getline(ss, item, ':'))
            model_paths_.push_back(item);
    }
}

// ----------------------------------------------------------------------------------------------------

ModelLoader::~ModelLoader()
{
}

// ----------------------------------------------------------------------------------------------------

std::string ModelLoader::getModelPath(const std::string& type) const
{
    for(std::vector<std::string>::const_iterator it = model_paths_.begin(); it != model_paths_.end(); ++it)
    {
        tue::filesystem::Path model_path(*it + "/" + type);
        if (model_path.exists())
            return model_path.string();
    }

    return "";
}

// ----------------------------------------------------------------------------------------------------

tue::config::DataConstPointer ModelLoader::loadModelData(const std::string& type, std::vector<std::string>& types, std::stringstream& error)
{
    std::map<std::string, ModelData>::iterator it = model_cache_.find(type);
    if (it != model_cache_.end())
    {
        types = it->second.second;
        const tue::config::DataConstPointer& data = it->second.first;
        return data;
    }

    tue::config::DataPointer data;

    std::string model_path = getModelPath(type);
    if (model_path.empty())
    {
        error << "ed::models::create() : Model '" << type << "' could not be found." << std::endl;
        return data;
    }

    bool sdf = true; //start with the assumption that we will find a sdf model
    tue::filesystem::Path model_cfg_path(model_path + "/model.sdf");
    if (!model_cfg_path.exists())
    {
        model_cfg_path = tue::filesystem::Path(model_path + "/model.yaml");
        sdf = false;
        if (!model_cfg_path.exists())
        {
            error << "ed::models::create() : ERROR loading configuration for model '" << type << "'; Both model.sdf or model.yaml file don't exist." << std::endl;
            return data;
        }
    }

    tue::Configuration model_cfg;
    if (sdf)
    {
        if (!model_cfg.loadFromSDFFile(model_cfg_path.string()))
        {
            error << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' failed to parse sdf file." << std::endl;
            return data;
        }
        if (!model_cfg.readGroup("sdf"))
        {
            error << "ed::models::create() : ERROR reading group 'SDF' in model '" << type << "'; '" << model_cfg_path.string() << "'." << std::endl;
            return tue::config::DataPointer();
        }
        if (!(model_cfg.readGroup("world") || model_cfg.readGroup("model")))
        {
            error << "ed::models::create() : ERROR reading group 'WORLD' or 'MODEL' in model '" << type << "'; '" << model_cfg_path.string() << "'." << std::endl;
            return tue::config::DataPointer();
        }
    }
    else
    {
        if (!model_cfg.loadFromYAMLFile(model_cfg_path.string()))
        {
            error << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' failed to parse yaml file." << std::endl;
            return data;
        }
    }

    if (sdf)
    {
        model_cfg.readGroup("sdf");
        model_cfg.readGroup("world");
        model_cfg.readGroup("model");
    }

    std::string super_type;
    if (model_cfg.value("type", super_type, tue::config::OPTIONAL) ||
            model_cfg.value("uri", super_type, tue::config::OPTIONAL) ||
            model_cfg.value("inherit", super_type, tue::config::OPTIONAL))
    {
        tue::config::DataConstPointer super_data = loadModelData(super_type, types, error);
        tue::config::DataPointer combined_data;
        combined_data.add(super_data);
        combined_data.add(model_cfg.data());

        types.push_back(super_type);

        data = combined_data;
    }
    else
    {
        data = model_cfg.data();
    }

    // If model loads a shape, set model path in shape data
    tue::config::ReaderWriter rw(data);
    if (rw.readGroup("shape") || rw.readArray("areas") || sdf) // always add the model path with an sdf. Because it is to deep to search in links for visuals/collisions
    {
        rw.setValue("__model_path__", model_path);
        if (!sdf)
            rw.endGroup(); // This is the same as endArray. Ugly but it works.
    }

    // Store data in cache
    model_cache_[type] = ModelData(data, types);

    return data;
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::exists(const std::string& type) const
{
    std::map<std::string, ModelData>::const_iterator it = model_cache_.find(type);
    if (it != model_cache_.end())
        return true;

    std::string model_path = getModelPath(type);
    return !model_path.empty();
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(const UUID& id, const std::string& type, UpdateRequest& req, std::stringstream& error)
{
    std::vector<std::string> types;
    tue::config::DataConstPointer data = loadModelData(type, types, error);
    if (data.empty())
        return false;

    if (!create(data, id, "", req, error))
        return false;

    types.push_back(type);
    for(std::vector<std::string>::const_iterator it = types.begin(); it != types.end(); ++it)
        req.addType(id, *it);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(const tue::config::DataConstPointer& data, UpdateRequest& req, std::stringstream& error)
{
    return create(data, "", "", req, error, "");
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id,
                         UpdateRequest& req, std::stringstream& error, const std::string& model_path,
                         const geo::Pose3D& pose_offset)
{
    tue::config::Reader r(data);

    // Get Id
    UUID id;
    std::string id_str;
    // force transition to id_opt in case of "_root". Only needed for SDF,  because then name is available for 'world' models. Which is not the case for ED YAML
    if (r.value("id", id_str, tue::config::OPTIONAL) || (r.value("name", id_str, tue::config::OPTIONAL) && (id_opt.str().empty() || id_opt.str()[0] != '_')))
    {
        if (parent_id.str().empty() || parent_id.str()[0] == '_')
            id = id_str;
        else
            id = parent_id.str() + "/" + id_str;
    }
    else if (!id_opt.str().empty())
    {
        id = id_opt;
    }
    else
    {
        id = ed::Entity::generateID();
    }


    // Get type. If it exists, first construct an entity based on the given type.

    std::string type;
    if (r.value("type", type, tue::config::OPTIONAL) || r.value("uri", type, tue::config::OPTIONAL) || r.value("inherit", type, tue::config::OPTIONAL)) //uri in sdf; inheret for inheritance in sdf
    {
        // remove prefix in case of sdf
        std::string str1 = "file://";
        std::string str2 = "model://";

        std::string::size_type i = type.find(str1);
        if (i != std::string::npos)
           type.erase(i, str1.length());
        i = type.find(str2);
        if (i != std::string::npos)
           type.erase(i, str2.length());

        std::vector<std::string> types;
        tue::config::DataConstPointer super_data = loadModelData(type, types, error);

        if (super_data.empty())
            return false;

        tue::config::DataPointer data_combined;
        data_combined.add(super_data);
        data_combined.add(data);

        r = tue::config::Reader(data_combined);

        types.push_back(type);
        for(std::vector<std::string>::const_iterator it = types.begin(); it != types.end(); ++it)
            req.addType(id, *it);
    }
    std::cout << "ALL DATA" << std::endl;
    std::cout << r.data() << std::endl;

    // Set type
    req.setType(id, type);

    // Get pose
    geo::Pose3D pose = geo::Pose3D::identity();
    if (!ed::models::readPose(r, pose))
    {
        return false;
    }

    // Apply pose offset
    pose = pose_offset * pose;

    req.setPose(id, pose);

    // Check the composition
    if (r.readArray("composition") || r.readArray("include"))
    {
        while (r.nextArrayItem())
            if (!create(r.data(), "", id, req, error, "", pose))
                return false;
        r.endArray();
    }


    // Set shape
    std::string shape_model_path = model_path;
    if (r.readGroup("shape"))
    {
        std::string shape_model_path = model_path;
        r.value("__model_path__", shape_model_path);

        geo::ShapePtr shape = loadShape(shape_model_path, r, shape_cache_, error);
        if (shape)
            req.setShape(id, shape);
        else
            return false;

        r.endGroup();
    }
    else if (r.value("__model_path__", shape_model_path)) // SDF
    {
        geo::CompositeShapePtr composite;
        std::map<std::string, geo::ShapePtr> dummy_shape_cache;
        if (r.readArray("link"))
        {
            while (r.nextArrayItem())
            {
                geo::Pose3D link_pose = geo::Pose3D::identity();
                readPose(r, link_pose);
                if (r.readArray("visual"))
                {
                    while(r.nextArrayItem())
                    {
                        readSDFGeometry(shape_model_path, r, composite, error, link_pose);
                    }
                    r.endArray();
                }
                geo::CompositeShapePtr area_composite;
                std::string area_name;
                if (r.value("name", area_name))
                {
                    if (r.readArray("virtual_area"))
                    {
                        while(r.nextArrayItem())
                        {
                            readSDFGeometry(shape_model_path, r, area_composite, error, link_pose);
                        }
                        r.endArray();
                    }
                    if (area_composite)
                        req.addArea(id, area_name, area_composite);
                 }
            }
            r.endArray();
        }
        if (composite)
            req.setShape(id, composite);
    }

    // Set areas
    if (r.readArray("areas"))
    {
        std::string shape_model_path = model_path;
        r.value("__model_path__", shape_model_path);
        while (r.nextArrayItem())
        {
            std::string area_name;
            if (!(r.value("name", area_name) && r.readArray("shape")))
                continue;

            geo::CompositeShapePtr shape;
            while (r.nextArrayItem())
            {
                geo::ShapePtr sub_shape = loadShape(shape_model_path, r, shape_cache_, error);
                if (sub_shape)
                {
                    if(!shape)
                        shape.reset(new geo::CompositeShape);
                    shape->addShape(*sub_shape, geo::Pose3D::identity());
                }
            }
            r.endArray();

            if (shape)
                req.addArea(id, area_name, shape);
        }
        r.endArray();
    }

    if (r.readArray("flags"))
    {
        while (r.nextArrayItem())
        {
            std::string flag;
            if (r.value("flag", flag))
                req.setFlag(id, flag);
        }
        r.endArray();
    }

    // Add additional data
    req.addData(id, r.data());

    return true;
}

} // end namespace models

} // end namespace ed

