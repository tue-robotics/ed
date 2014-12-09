#ifndef WIRE_VOLUME_MODEL_DB_LOADER_H_
#define WIRE_VOLUME_MODEL_DB_LOADER_H_

#include <geolib/datatypes.h>
#include <tue/config/configuration.h>

namespace ed
{

namespace models
{

geo::ShapePtr loadShape(const std::string& model_path, tue::Configuration cfg);

} // end models namespace

} // end ed namespace

#endif
