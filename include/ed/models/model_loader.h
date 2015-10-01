#ifndef ED_MODEL_LOADER_H_
#define ED_MODEL_LOADER_H_

#include "ed/uuid.h"

#include <map>
#include <geolib/datatypes.h>
#include <tue/config/data_pointer.h>

namespace ed
{

namespace models
{

class ModelLoader
{

public:

    ModelLoader();

    ~ModelLoader();

    bool create(const UUID& id, const std::string& type, UpdateRequest& req, std::stringstream& error);

    bool create(const tue::config::DataConstPointer& data, UpdateRequest& req, std::stringstream& error);

    bool create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id,
                UpdateRequest& req, std::stringstream& error, const std::string& model_path = "",
                const geo::Pose3D& pose_offset = geo::Pose3D::identity());

    bool exists(const std::string& type) const;

private:

    typedef std::pair<tue::config::DataConstPointer, std::vector<std::string> > ModelData;

    // Model name to model data
    std::map<std::string, ModelData> model_cache_;

    // Shape filename to shape
    std::map<std::string, geo::ShapePtr> shape_cache_;

    std::vector<std::string> model_paths_;

    tue::config::DataConstPointer loadModelData(const std::string& type, std::vector<std::string>& types, std::stringstream& error);

    std::string getModelPath(const std::string& type) const;

};

} // end namespace models

} // end namespace ed

#endif
