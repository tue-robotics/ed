#include "ed/models/models.h"
#include "ed/update_request.h"
#include "ed/relations/transform_cache.h"

#include <tue/filesystem/path.h>
#include <ros/package.h>

#include "shape_loader.h"

#include <tue/config/reader.h>
#include <tue/config/writer.h>

namespace ed
{

namespace models
{

// ----------------------------------------------------------------------------------------------------

tue::filesystem::Path getModelPath(const std::string& type)
{
    return tue::filesystem::Path(ros::package::getPath("ed_object_models") + "/models/" + type);
}

// ----------------------------------------------------------------------------------------------------

tue::config::DataPointer loadModelData(const std::string& type, std::string& model_path_str)
{
    tue::config::DataPointer data;

    tue::filesystem::Path model_path = getModelPath(type);
    if (!model_path.exists())
    {
        std::cout << "ed::models::create() : ERROR loading model '" << type << "'; '" << model_path.string() << "' does not exist." << std::endl;
        return data;
    }

    tue::filesystem::Path model_cfg_path(model_path.string() + "/model.yaml");
    if (!model_cfg_path.exists())
    {
        std::cout << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' file does not exist." << std::endl;
        return data;
    }

    tue::Configuration model_cfg;
    if (!model_cfg.loadFromYAMLFile(model_cfg_path.string()))
    {
        std::cout << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' failed to parse yaml file." << std::endl;
        return data;
    }

    model_path_str = model_path.string();
    return model_cfg.data();
}

// ----------------------------------------------------------------------------------------------------

bool create(const UUID& id, const std::string& type, UpdateRequest& req)
{
    std::string model_path;
    tue::config::DataPointer data = loadModelData(type, model_path);
    if (!data.valid())
        return false;

    if (!create(data, id, "", req, model_path))
        return false;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id, UpdateRequest& req, const std::string& model_path)
{
    tue::config::Reader r(data);

    // Get Id
    UUID id;
    std::string id_str;
    if (r.value("id", id_str, tue::config::OPTIONAL))
    {
        if (id_opt.str().empty())
            id = id_str;
        else
            id = id_opt.str() + "/" + id_str;
    }
    else
        id = id_opt;

    // Get type. If it exists, first construct an entity based on the given type.
    std::string type;
    if (r.value("type", type, tue::config::OPTIONAL))
    {
        std::string super_model_path;
        tue::config::DataPointer super_data = loadModelData(type, super_model_path);
        if (super_data.valid())
            create(super_data, id, parent_id, req, super_model_path);
    }

    // Set type
    req.setType(id, type);

    // Get pose
    if (r.readGroup("pose"))
    {
        geo::Pose3D pose;

        if (r.value("x", pose.t.x) && r.value("y", pose.t.y) && r.value("z", pose.t.z))
        {
            double rx = 0, ry = 0, rz = 0;
            r.value("X", rx, tue::config::OPTIONAL);
            r.value("Y", ry, tue::config::OPTIONAL);
            r.value("Z", rz, tue::config::OPTIONAL);

            // Set rotation
            pose.R.setRPY(rx, ry, rz);

            std::string parent_id_str;
            if (!r.value("parent", parent_id_str, tue::config::OPTIONAL))
                parent_id_str = parent_id.str();

            boost::shared_ptr<ed::TransformCache> transform(new ed::TransformCache());
            transform->insert(Time(-1), pose);  // TODO: choose proper time
            req.setRelation(parent_id_str, id, transform);
        }

        r.endGroup();
    }

    // Check the composition
    if (r.readArray("composition"))
    {
        while (r.nextArrayItem())
        {
            create(r.data(), id, id, req);
        }

        r.endArray();
    }

    // Set shape
    if (r.readGroup("shape"))
    {
        if (!model_path.empty() && tue::filesystem::Path(model_path).exists())
        {
            geo::ShapePtr shape = loadShape(model_path, r);
            if (shape)
                req.setShape(id, shape);
        }

        r.endGroup();
    }

    // Add additional data
    req.addData(id, data);
}

} // namespace models

} // namespace ed
