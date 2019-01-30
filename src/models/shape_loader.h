#ifndef ED_MODELS_SHAPE_LOADER_H_
#define ED_MODELS_SHAPE_LOADER_H_

#include <geolib/datatypes.h>
#include <tue/config/reader.h>

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

std::vector<std::string> split(std::string& strToSplit, char delimeter);

std::string parseURI(const std::string& uri, ModelOrFile& uri_type);

geo::ShapePtr loadShape(const std::string& model_path, tue::config::Reader cfg,
                        std::map<std::string, geo::ShapePtr>& shape_cache, std::stringstream& error);

geo::ShapePtr getHeightMapShape(const std::string& image_filename, const geo::Vec3& pos, const double blockheight,
                                const double resolution_x, const double resolution_y, const bool inverted, std::stringstream& error);

bool readPose(tue::config::Reader& cfg, geo::Pose3D& pose,
              tue::config::RequiredOrOptional pos_req = tue::config::REQUIRED,
              tue::config::RequiredOrOptional rot_req = tue::config::OPTIONAL);

void createPolygon(geo::Shape& shape, const std::vector<geo::Vec2>& points, double height, std::stringstream& error, bool create_bottom = true);

void createCylinder(geo::Shape& shape, double radius, double height, int num_corners = 12);

void createSphere(geo::Shape& shape, double radius, int recursion_level = 2);

} // end models namespace

} // end ed namespace

#endif
