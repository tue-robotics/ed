#include "ed/models/models.h"
#include "ed/update_request.h"
#include "ed/relations/transform_cache.h"

#include <tue/filesystem/path.h>
#include <ros/package.h>

#include "shape_loader.h"

#include <tue/config/reader.h>

namespace ed
{

namespace models
{

bool create(UpdateRequest& req, const UUID& id, const TYPE& type, tue::Configuration cfg)
{
    tue::filesystem::Path model_path(ros::package::getPath("ed_object_models") + "/models/" + type);
    if (!model_path.exists())
    {
        std::cout << "ed::models::create() : ERROR loading model '" << type << "'; '" << model_path.string() << "' does not exist." << std::endl;
        return false;
    }

    tue::filesystem::Path model_cfg_path(model_path.string() + "/model.yaml");
    if (!model_cfg_path.exists())
    {
        std::cout << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' file does not exist." << std::endl;
        return false;
    }

    tue::Configuration model_cfg;
    if (!model_cfg.loadFromYAMLFile(model_cfg_path.string()))
    {
        std::cout << "ed::models::create() : ERROR loading configuration for model '" << type << "'; '" << model_cfg_path.string() << "' failed to parse yaml file." << std::endl;
        return false;
    }

    //! Check the inheritance
    std::string parent_type;
    if (model_cfg.value("type", parent_type, tue::OPTIONAL))
        create(req, id, parent_type, tue::Configuration());

    req.setType(id, type);

    //! Check the composition
    if (model_cfg.readArray("composition"))
    {
        while (model_cfg.nextArrayItem())
        {
            std::string child_id, child_type;
            if (model_cfg.value("type", child_type) && model_cfg.value("id", child_id))
            {
                create(req, child_id, child_type, model_cfg.limitScope());

                geo::Pose3D pose;

                //! Set pose
                if (model_cfg.readGroup("pose"))
                {
                    if (model_cfg.value("x", pose.t.x) && model_cfg.value("y", pose.t.y) && model_cfg.value("z", pose.t.z))
                    {
                        double rx = 0, ry = 0, rz = 0;
                        model_cfg.value("X", rx, tue::OPTIONAL);
                        model_cfg.value("Y", ry, tue::OPTIONAL);
                        model_cfg.value("Z", rz, tue::OPTIONAL);

                        pose.R.setRPY(rx, ry, rz);
                    }

                    model_cfg.endGroup();
                }
                else
                {
                    pose = geo::Pose3D::identity();
                }

                boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());

                // TODO: choose proper time
                t1->insert(Time(-1), pose);
                req.setRelation(id, child_id, t1);
            }
        }

        model_cfg.endArray();
    }

    //! Set shape
    if (model_cfg.readGroup("shape"))
    {
        geo::ShapePtr shape = loadShape(model_path.string(), model_cfg.limitScope());
        if (shape)
            req.setShape(id, shape);

        model_cfg.endGroup();
    }

    // Add additional data
    req.addData(id, model_cfg.data());
    req.addData(id, cfg.data());

    return true;
}

} // namespace models

} // namespace ed
