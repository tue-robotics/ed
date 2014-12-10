#ifndef WIRE_VOLUME_MODEL_DB_LOADER_H_
#define WIRE_VOLUME_MODEL_DB_LOADER_H_

#include <geolib/datatypes.h>
#include <tue/config/reader.h>

namespace ed
{

namespace models
{

geo::ShapePtr loadShape(const std::string& model_path, tue::config::Reader cfg);

} // end models namespace

} // end ed namespace

#endif
