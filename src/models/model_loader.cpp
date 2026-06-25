#include "ed/models/model_loader.h"

#include "ed/types.h"

#include "ed/entity.h"
#include "ed/update_request.h"

#include <boost/smart_ptr/shared_ptr.hpp>
#include <cstdlib>
#include <filesystem>

#include "shape_loader_private.h"

#include <geolib/datatypes.h>
#include <map>
#include <ostream>
#include <tue/config/configuration.h>
#include <tue/config/data_pointer.h>
#include <tue/config/reader.h>
#include <tue/config/reader_writer.h>
#include <tue/config/types.h>

#include <geolib/CompositeShape.h>

#include <memory>
#include <sdf/parser.hh>

#include <sstream>
#include <vector>

namespace ed::models
{

static bool readSDFGeometry(tue::config::Reader r,
                            geo::CompositeShapePtr& composite,
                            std::stringstream& error,
                            const geo::Pose3D& pose_offset = geo::Pose3D::identity())
{
    geo::Pose3D pose = geo::Pose3D::identity();
    readPose(r, pose);
    pose = pose_offset * pose;
    if (!r.readGroup("geometry"))
        return false;

    std::map<std::string, geo::ShapePtr> dummy_shape_cache;
    geo::ShapePtr const sub_shape = loadShape("", r, dummy_shape_cache, error);
    if (sub_shape)
    {
        if (!composite) // if pointer is empty, create new instance.
            composite = std::make_shared<geo::CompositeShape>();
        composite->addShape(*sub_shape, pose);
    }
    r.endGroup();
    return true;
}

// ----------------------------------------------------------------------------------------------------

ModelLoader::ModelLoader()
{
    const char* edmpath = ::getenv("ED_MODEL_PATH");
    if (edmpath)
    {
        std::vector<std::string> const paths_vector = ed::models::split(edmpath, ':');
        for (const auto& it : paths_vector)
            ed_model_paths_.push_back(it);
    }
    const char* mpath = ::getenv("GAZEBO_MODEL_PATH");
    if (mpath)
    {
        std::vector<std::string> const paths_vector = ed::models::split(mpath, ':');
        for (const auto& it : paths_vector)
            model_paths_.push_back(it);
    }
    const char* fpath = ::getenv("GAZEBO_RESOURCE_PATH");
    if (fpath)
    {
        std::vector<std::string> const paths_vector = ed::models::split(fpath, ':');
        for (const auto& it : paths_vector)
            file_paths_.push_back(it);
    }
}

// ----------------------------------------------------------------------------------------------------

ModelLoader::~ModelLoader() = default;

// ----------------------------------------------------------------------------------------------------

/**
 * @brief ModelLoader::getModelPath
 * @param type type
 * @return The full path or empty string, if not found
 */

std::string ModelLoader::getModelPath(const std::string& type) const
{
    for (const auto& ed_model_path : ed_model_paths_)
    {
        std::filesystem::path const model_path(ed_model_path + "/" + type);
        if (std::filesystem::exists(model_path))
            return model_path.string();
    }

    return "";
}

// ----------------------------------------------------------------------------------------------------

std::string ModelLoader::getSDFPath(const std::string& uri) const
{
    ModelOrFile uri_type;
    std::string const parsed_uri = parseURI(uri, uri_type);
    if (parsed_uri.empty())
        return "";

    if (uri_type == MODEL)
    {
        for (const auto& it : model_paths_)
        {
            std::filesystem::path const model_dir(it + "/" + parsed_uri);
            if (std::filesystem::exists(model_dir))
            {
                std::filesystem::path const config_path(model_dir.string() + "/model.config");
                if (std::filesystem::exists(config_path))
                {
                    std::filesystem::path const model_path = sdf::getModelFilePath(model_dir.string());
                    if (std::filesystem::exists(model_path))
                        return model_path.string();
                }
            }
        }
    }
    if (uri_type == FILE)
    {
        for (const auto& it : file_paths_)
        {
            std::filesystem::path const file_path(it + "/" + parsed_uri);
            if (std::filesystem::exists(file_path))
                return file_path.string();
        }
    }

    return "";
}

// ----------------------------------------------------------------------------------------------------

ModelLoader::ModelData ModelLoader::readModelCache(const std::string& type) const
{
    auto const it = model_cache_.find(type);
    if (it != model_cache_.end())
        return it->second;

    tue::config::DataConstPointer const data;
    std::vector<std::string> const types;
    return ModelData(data, types);
}

// ----------------------------------------------------------------------------------------------------

tue::config::DataConstPointer ModelLoader::loadModelData(const std::string& type,
                                                         std::vector<std::string>& types,
                                                         std::stringstream& error,
                                                         const bool allow_sdf)
{
    if (allow_sdf)
    {
        tue::config::DataConstPointer data_sdf;
        data_sdf = loadSDFData("model://" + type, error);
        if (!data_sdf.empty())
            return data_sdf;
    }
    ModelData const cache_data = readModelCache(type);
    if (!cache_data.first.empty())
    {
        types = cache_data.second;
        return cache_data.first;
    }

    tue::config::DataPointer data;

    std::string const model_path = getModelPath(type);
    if (model_path.empty())
    {
        error << "[ed::models::loadModelData] Model '" << type << "' could not be found." << '\n';
        return data;
    }

    std::filesystem::path const model_cfg_path(model_path + "/model.yaml");
    if (!std::filesystem::exists(model_cfg_path))
    {
        error << "[ed::models::loadModelData] ERROR loading configuration for model '" << type << "'; '"
              << model_cfg_path.string() << "' file does not exist." << '\n';
        return data;
    }

    tue::Configuration model_cfg;
    if (!model_cfg.loadFromYAMLFile(model_cfg_path.string()))
    {
        error << "[ed::models::loadModelData] ERROR loading configuration for model '" << type << "'; '"
              << model_cfg_path.string() << "' failed to parse yaml file." << '\n';
        return data;
    }

    std::string super_type;
    if (model_cfg.value("type", super_type, tue::config::OPTIONAL))
    {
        tue::config::DataConstPointer const super_data = loadModelData(super_type, types, error);
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
    rw.setValue("__model_path__", model_path);

    // Store data in cache
    model_cache_[type] = ModelData(data, types);

    return data;
}

// ----------------------------------------------------------------------------------------------------

tue::config::DataConstPointer ModelLoader::loadSDFData(const std::string& uri, std::stringstream& error)
{
    tue::config::DataPointer data;
    ModelOrFile uri_type;
    std::string const parsed_uri = parseURI(uri, uri_type);
    if (parsed_uri.empty())
    {
        error << "[ed::models::loadSDFData] Incorrect URI: '" << uri << "'." << '\n';
        return data;
    }
    ModelData const cache_data = readModelCache(parsed_uri + "_sdf");
    if (!cache_data.first.empty())
    {
        return cache_data.first;
    }

    std::filesystem::path const model_cfg_path = getSDFPath(uri);
    if (!std::filesystem::exists(model_cfg_path))
    {
        error << "[ed::models::loadSDFData] Model '" << uri << "' could not be found." << '\n';
        return data;
    }

    tue::Configuration model_cfg(data);
    if (!model_cfg.loadFromSDFFile(model_cfg_path.string()))
    {
        error << "[ed::models::loadSDFData] ERROR loading configuration for model '" << uri << "'; '" << model_cfg_path
              << "' failed to parse SDF file." << '\n';
        error << model_cfg.error() << '\n';
        return data;
    }

    // Store data in cache
    model_cache_[parsed_uri + "_sdf"] = ModelData(data, std::vector<std::string>());

    return data;
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::exists(const std::string& type) const
{
    ModelData cache_data = readModelCache(type + "_sdf");
    if (!cache_data.first.empty())
        return true;

    cache_data = readModelCache(type);
    if (!cache_data.first.empty())
        return true;

    std::string const sdf_path = getSDFPath(type);
    if (!sdf_path.empty())
        return true;

    std::string const model_path = getModelPath(type);
    return !model_path.empty();
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(
    const UUID& id, const std::string& type, UpdateRequest& req, std::stringstream& error, const bool allow_sdf)
{
    tue::config::DataConstPointer data;
    std::vector<std::string> types;
    bool sdf = true;
    if (allow_sdf)
        data = loadSDFData("model://" + type, error);
    if (data.empty())
    {
        sdf = false;
        data = loadModelData(type, types, error);
        if (data.empty())
            return false;
    }

    // First try to get data before trying to create models.
    if (sdf)
    {
        if (!createSDF(data, "", geo::Pose3D::identity(), id, boost::shared_ptr<const geo::Pose3D>(), req, error))
            return false;
    }
    else
    {
        if (!create(data, id, "", req, error))
            return false;
    }

    types.push_back(type);
    for (const auto& type : types)
        req.addType(id, type);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(const tue::config::DataConstPointer& data, UpdateRequest& req, std::stringstream& error)
{
    return create(data, "_root", "", req, error, "");
}

// ----------------------------------------------------------------------------------------------------

bool ModelLoader::create(const tue::config::DataConstPointer& data,
                         const UUID& id_opt,
                         const UUID& parent_id,
                         UpdateRequest& req,
                         std::stringstream& error,
                         const std::string& model_path,
                         const geo::Pose3D& pose_offset)
{
    tue::config::Reader r(data);

    if (r.hasGroup("sdf"))
        return createSDF(r.data(), parent_id, pose_offset, id_opt, boost::shared_ptr<const geo::Pose3D>(), req, error);

    // Get Id
    UUID id;
    std::string id_str;
    if (r.value("id", id_str, tue::config::OPTIONAL))
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
    if (r.value("type", type, tue::config::OPTIONAL))
    {
        std::vector<std::string> types;
        tue::config::DataConstPointer const super_data = loadModelData(type, types, error);

        if (super_data.empty())
            return false;

        tue::config::DataPointer data_combined;
        data_combined.add(super_data);
        data_combined.add(data);

        r = tue::config::Reader(data_combined);

        types.push_back(type);
        for (const auto& type : types)
            req.addType(id, type);
    }

    // Set type
    req.setType(id, type);

    // Get pose
    geo::Pose3D pose = geo::Pose3D::identity();
    if (!ed::models::readPose(r, pose))
        error << "[ed::models::create] No pose, while reading model: '" << id << "'" << '\n';

    pose = pose_offset * pose;

    req.setPose(id, pose);

    // Check the composition
    if (r.readArray("composition"))
    {
        while (r.nextArrayItem())
        {
            if (!create(r.data(), "", id, req, error, "", pose))
                return false;
        }

        r.endArray();
    }

    std::string shape_model_path = model_path;
    r.value("__model_path__", shape_model_path);
    // Set shape
    if (r.readGroup("shape"))
    {
        geo::ShapePtr const shape = loadShape(shape_model_path, r, shape_cache_, error);
        if (shape)
        {
            req.setVisual(id, shape);
            req.setCollision(id, shape);
        }
        else
            return false;

        r.endGroup();
    }

    // Set volumes
    if (r.readArray("areas") || r.readArray("volumes"))
    {
        while (r.nextArrayItem())
        {
            std::string volume_name;
            if (!(r.value("name", volume_name) && r.readArray("shape")))
                continue;

            geo::CompositeShapePtr shape;
            while (r.nextArrayItem())
            {
                geo::ShapePtr const sub_shape = loadShape(shape_model_path, r, shape_cache_, error);
                if (sub_shape)
                {
                    if (!shape)
                        shape = std::make_shared<geo::CompositeShape>();
                    shape->addShape(*sub_shape, geo::Pose3D::identity());
                }
            }
            r.endArray();

            if (shape)
                req.addVolume(id, volume_name, shape);
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

bool ModelLoader::createSDF(const tue::config::DataConstPointer& data,
                            const UUID& parent_id,
                            const geo::Pose3D& parent_pose,
                            const UUID& id_override,
                            const boost::shared_ptr<const geo::Pose3D>& pose_override,
                            UpdateRequest& req,
                            std::stringstream& error)
{
    tue::config::Reader r(data);

    r.readGroup("sdf"); // Just read the sdf element

    bool const sdf_world = r.readGroup("world");
    bool sdf_model = false;
    if (!sdf_world)
    {
        sdf_model = r.readArray("model");
        r.nextArrayItem();
    }

    if (!sdf_world && !sdf_model)
    {
        error << "[ed::models::createSDF] Not a valid SDF model, because no 'world' or 'model' available: " << '\n'
              << data << '\n';
        return false;
    }

    if (r.nextArrayItem())
    {
        error << "[ed::models::createSDF] A model sdf file should only contain one model." << '\n';
        return false;
    }

    // ID
    UUID id;
    std::string id_str;
    if (!id_override.str().empty())
        id = id_override.str();
    else if (r.value("name", id_str) && !id_str.empty())
        id = id_str;
    else
        id = ed::Entity::generateID();

    // prefix with parent_id
    if (!parent_id.str().empty() && parent_id.str()[0] != '_')
        id = parent_id.str() + "/" + id.str();

    // pose
    geo::Pose3D pose = geo::Pose3D::identity();
    readPose(r, pose);
    if (pose_override)
        pose = *pose_override;
    pose = parent_pose * pose;
    req.setPose(id, pose);

    // set type if defined, no recursive type is done.
    std::string type;
    if (r.value("type", type))
        req.setType(id, type);

    // composed models
    if (r.readArray("model"))
    {
        while (r.nextArrayItem())
        {
            tue::config::ReaderWriter rw;
            rw.writeArray("model");
            rw.addArrayItem();
            rw.data().add(r.data());
            rw.endArray();
            if (!createSDF(rw.data(), id, pose, "", boost::shared_ptr<const geo::Pose3D>(), req, error))
                return false;
        }
        r.endArray(); // end array model
    }
    if (r.readArray("include"))
    {
        while (r.nextArrayItem())
        {
            std::string child_id;
            geo::Pose3D child_pose;
            std::string uri;
            ed::shared_ptr<const geo::Pose3D> child_posePtr;

            r.value("name", child_id);
            if (readPose(r, child_pose))
                child_posePtr = ed::make_shared<const geo::Pose3D>(child_pose);
            if (!r.value("uri", uri))
            {
                error << "No uri found for include in model: '" << id << "'." << '\n' << r.data() << '\n';
                return false;
            }

            std::vector<std::string> const types;
            tue::config::DataConstPointer const child_data = loadSDFData(uri, error);
            if (!createSDF(child_data, id, pose, child_id, child_posePtr, req, error))
                return false;
        }
        r.endArray(); // end array include
    }

    // visual, collision & volumes
    geo::CompositeShapePtr visual_composite;
    geo::CompositeShapePtr collision_composite;
    std::map<std::string, geo::ShapePtr> const dummy_shape_cache;
    if (r.readArray("link"))
    {
        while (r.nextArrayItem())
        {
            geo::Pose3D link_pose = geo::Pose3D::identity();
            readPose(r, link_pose);
            if (r.readArray("visual"))
            {
                while (r.nextArrayItem())
                {
                    readSDFGeometry(r, visual_composite, error, link_pose);
                }
                r.endArray();
            }
            if (r.readArray("collision"))
            {
                while (r.nextArrayItem())
                {
                    readSDFGeometry(r, collision_composite, error, link_pose);
                }
                r.endArray();
            }
            geo::CompositeShapePtr volume_composite;
            std::string volume_name;
            if (r.value("name", volume_name))
            {
                if (r.readArray("virtual_volume"))
                {
                    while (r.nextArrayItem())
                    {
                        readSDFGeometry(r, volume_composite, error, link_pose);
                    }
                    r.endArray();
                }
                if (volume_composite)
                    req.addVolume(id, volume_name, volume_composite);
            }
        }
        r.endArray(); // end array link
    }
    if (visual_composite)
        req.setVisual(id, visual_composite);
    if (collision_composite)
        req.setCollision(id, collision_composite);

    if (sdf_world)
        r.endGroup(); // end group world
    else // sdf_model
        r.endArray(); // end array model

    return true;
}

} // namespace ed::models

// end namespace ed
