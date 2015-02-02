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

    bool create(const UUID& id, const std::string& type, UpdateRequest& req);

    bool create(const tue::config::DataConstPointer& data, UpdateRequest& req);

    bool create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id, UpdateRequest& req, const std::string& model_path = "");

private:

    // Model name to model data
    std::map<std::string, tue::config::DataConstPointer> model_cache_;

    // Shape filename to shape
    std::map<std::string, geo::ShapePtr> shape_cache_;

    tue::config::DataConstPointer loadModelData(const std::string& type, std::string& model_path_str);

    tue::config::DataConstPointer loadModelData(const std::string& type, std::string& model_path_str, std::stringstream& error);

};

} // end namespace models

} // end namespace ed

#endif
