#ifndef ED_MODELS_SHAPE_LOADER_PRIVATE_H_
#define ED_MODELS_SHAPE_LOADER_PRIVATE_H_

// Some functions are moved to the include folder. So these can be included in other packages. There has been no need for the remaining
// functions in this file to be used in other packages. Therefore, these are kept here. But there is no reason for the functions not to
// be moved, if needed to be include somewhere else.

#include <geolib/datatypes.h>
#include <tue/config/reader.h>

#include <map>
#include <string>
#include <vector>
#include <sstream>

namespace ed
{

namespace models
{

/**
 * @brief The ModelOrFile enum This is used to determine the URI type in SDF.
 */
enum ModelOrFile
{
    MODEL,
    FILE
};

/**
 * @brief split Implementation by using delimiter as a character. Multiple delimeters are removed.
 * @param strToSplit input string, which is splitted
 * @param delimeter char on which the string is split
 * @return vector of sub-strings
 */
std::vector<std::string> split(const std::string& strToSplit, char delimeter);

std::string parseURI(const std::string& uri, ModelOrFile& uri_type);

/**
 * @brief loadShape load the shape of a model.
 * @param model_path path of the model
 * @param cfg reader
 * @param shape_cache cache for complex models
 * @param error errorstream
 * @return final mesh; or empty mesh in case of error
 */
geo::ShapePtr loadShape(const std::string& model_path, tue::config::Reader cfg,
                        std::map<std::string, geo::ShapePtr>& shape_cache, std::stringstream& error);

/**
 * @brief getHeightMapShape convert grayscale image in a heigtmap mesh
 * @param image_filename full path of grayscale image
 * @param pos position of the origin of the heigtmap
 * @param blockheight height of the heightmap of max grayscale value
 * @param resolution_x resolution in x direction in meters
 * @param resolution_y resolution in y direction in meters
 * @param inverted false: CV/ROS standard (black = height); true: SDF/GAZEBO (White = height)
 * @param errorerrorstream
 * @return final mesh; or empty mesh in case of error
 */
geo::ShapePtr getHeightMapShape(const std::string& image_filename, const geo::Vec3& pos, const double blockheight,
                                const double resolution_x, const double resolution_y, const bool inverted, std::stringstream& error);

/**
 * @brief readPose read pose into Pose3D. Both ED yaml and SDF. Also reads pos(position) of SDF.
 * @param cfg reader
 * @param pose filled Pose3D pose
 * @param pos_req position RequiredOrOptional
 * @param rot_req rotation RequiredOrOptional
 * @return indicates succes
 */
bool readPose(tue::config::Reader& cfg, geo::Pose3D& pose,
              tue::config::RequiredOrOptional pos_req = tue::config::REQUIRED,
              tue::config::RequiredOrOptional rot_req = tue::config::OPTIONAL);

/**
 * @brief createPolygon create polygon mesh from points
 * @param shape filled mesh
 * @param points 2D points which define the mesh
 * @param height height of the mesh
 * @param error error stream
 * @param create_bottom false: open bottom; true: closed bottom
 */
void createPolygon(geo::Shape& shape, const std::vector<geo::Vec2>& points, double height, std::stringstream& error, bool create_bottom = true);

} // end models namespace

} // end ed namespace

#endif
