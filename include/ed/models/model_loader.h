#ifndef ED_MODEL_LOADER_H_
#define ED_MODEL_LOADER_H_

#include "ed/uuid.h"

#include <geolib/datatypes.h>
#include <tue/config/data_pointer.h>

#include <map>
#include <vector>

namespace ed
{

namespace models
{

class ModelLoader
{

public:

    ModelLoader();

    ~ModelLoader();

    /**
     * @brief create add entity to update_request with id 'id' of the type 'type'. If allow_sdf is true, also sdf models
     * are considered. Otherwise only ED yaml models are used.
     * @param id new id of the entity
     * @param type type of the entity
     * @param req UpdateRequest to 'fill'
     * @param error error stream
     * @param allow_sdf Allow SDF models, or only ED yaml models
     * @return bool, which indicates succes
     */
    bool create(const UUID& id, const std::string& type, UpdateRequest& req, std::stringstream& error, const bool allow_sdf=false);

    /**
     * @brief create add entity to update_request from config data. "_root" will be used as id.
     * @param data should contain the right data and should be at the correct reading point
     * @param req UpdateRequest to 'fill'
     * @param error error stream
     * @return bool, which indicates succes
     */
    bool create(const tue::config::DataConstPointer& data, UpdateRequest& req, std::stringstream& error);

    /**
     * @brief create add entity to update_request from config data.
     * @param data should contain the right data and should be at the correct reading point
     * @param id_opt Optional id, if no id in data
     * @param parent_id prefix to id, unless starting with "_"
     * @param req UpdateRequest to 'fill'
     * @param error error stream
     * @param model_path path where the data is read. Is used for relative paths in the data
     * @param pose_offset pose offset, if no pose in data, this pose will be the final pose.
     * @return bool, which indicates succes
     */
    bool create(const tue::config::DataConstPointer& data, const UUID& id_opt, const UUID& parent_id,
                UpdateRequest& req, std::stringstream& error, const std::string& model_path = "",
                const geo::Pose3D& pose_offset = geo::Pose3D::identity());

    /**
     * @brief createSDF add SDF entity to update_request from config data.
     * @param data should contain the right data and should be at the correct reading point
     * @param parent_id prefix to id, unless starting with "_"
     * @param parent_pose pose offset which will always be applied
     * @param id_override if not empty, this will override the id of the model
     * @param pose_override if not empty, this will override the pose of the model
     * @param req UpdateRequest to 'fill'
     * @param error error stream
     * @return bool, which indicates succes
     */
    bool createSDF(const tue::config::DataConstPointer& data, const UUID& parent_id, const geo::Pose3D& parent_pose,
                   const UUID& id_override, const boost::shared_ptr<const geo::Pose3D> pose_override,
                   UpdateRequest& req, std::stringstream& error);

    /**
     * @brief exists Check of a model of type 'type' exist
     * @param type model type
     * @return bool, which indicates of type exist
     */
    bool exists(const std::string& type) const;

private:

    typedef std::pair<tue::config::DataConstPointer, std::vector<std::string> > ModelData;

    // Model name to model data
    std::map<std::string, ModelData> model_cache_;

    // Shape filename to shape
    std::map<std::string, geo::ShapePtr> shape_cache_;

    // Vectors which contain the paths, where models can be found
    std::vector<std::string> ed_model_paths_; // ED_MODEL_PATH
    std::vector<std::string> model_paths_; // GAZEBO_MODEL_PATH
    std::vector<std::string> file_paths_; // GAZEBO_RESOURCE_PATH

    /**
     * @brief loadModelData load data of model of type 'type'
     * @param type type of the model
     * @param types vector of model types for recursive inheritance
     * @param error error stream
     * @param allow_sdf Allow SDF models, or only ED yaml models
     * @return DataConstPointer with the data, empty in case of error
     */
    tue::config::DataConstPointer loadModelData(std::string type, std::vector<std::string>& types,
                                                std::stringstream& error, const bool allow_sdf=false);

    /**
     * @brief loadSDFData load data of SDF model of uri 'uri'
     * @param uri uri of the model
     * @param error error stream
     * @return DataConstPointer with the data, empty in case of error
     */
    tue::config::DataConstPointer loadSDFData(std::string uri, std::stringstream& error);


    /**
     * @brief getModelPath get file path of model of type 'type'
     * @param type type of the model
     * @return path of the model, empty if model not found
     */
    std::string getModelPath(const std::string& type) const;

    /**
     * @brief getSDFPath get file path of SDF model with uri 'uri'
     * @param uri uri of the model
     * @return path of the model, empty if model not found
     */
    std::string getSDFPath(const std::string& uri) const;

    /**
     * @brief readModelCache read the model cache, which contains the data and all types
     * @param type type of the model to look for
     * @return ModelData, DataPointer is empty in case type not in cache
     */
    ModelData readModelCache(std::string type) const;

};


/**
 * @brief The LoadType enum indicates whether to load directly from a file
 * or from a model that is part of the ED_MODEL_PATH
 */
enum class LoadType
{
   FILE,
   MODEL,
};


/**
 * @brief loadModel loads an ED model from file
 * @param load_type indicates whether the provided source is a filename or an identifier
 * of a model in the ED_MODEL_PATH
 * @param source source filename or entity type
 * @param req update request that will be filled with the data from the model
 * @return success
 */
bool loadModel(const LoadType load_type, const std::string& source, ed::UpdateRequest& req);

} // end namespace models

} // end namespace ed

#endif
