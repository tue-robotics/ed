#ifndef WIRE_VOLUME_MODEL_DB_XML_SHAPE_PARSER_H_
#define WIRE_VOLUME_MODEL_DB_XML_SHAPE_PARSER_H_

#include <geolib/datatypes.h>

geo::ShapePtr parseXMLShape(const std::string& filename, std::string& error);

#endif
