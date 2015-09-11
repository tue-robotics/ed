#ifndef ED_SERIALIZATION_H_
#define ED_SERIALIZATION_H_

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <geolib/datatypes.h>

namespace tue {
namespace config {
class Reader;
class Writer;
}
}

namespace ed
{

class WorldModel;
class Entity;
class UpdateRequest;
class ConvexHull;
class ImageMask;

namespace io
{
class Reader;
class Writer;
}
}

namespace ed
{

// SERIALIZATION

//void serialize(const WorldModel& wm, ed::io::Writer& w, unsigned long since_revision = 0);


//void serialize(const Entity& wm, ed::io::Writer& w, unsigned long since_revision = 0);


bool deserialize(io::Reader &r, UpdateRequest& req);


void serialize(const geo::Pose3D& pose, ed::io::Writer& w);

bool deserialize(ed::io::Reader& r, geo::Pose3D& pose);

bool deserialize(tue::config::Reader& r, const std::string& group, geo::Pose3D& pose);

bool deserialize(tue::config::Reader& r, const std::string& group, geo::Vec3& p);


void serialize(const ConvexHull& ch, ed::io::Writer& w);

bool deserialize(ed::io::Reader& r, ConvexHull& ch);


void serialize(const geo::Shape& s, ed::io::Writer& w);

bool deserialize(ed::io::Reader& r, geo::Shape& s);

bool deserialize(tue::config::Reader& r, const std::string& group, geo::Shape& s);


void serializeTimestamp(double time, ed::io::Writer& w);

bool deserializeTimestamp(ed::io::Reader& r, double& time);


void serialize(const ImageMask& mask, tue::serialization::OutputArchive& m);

bool deserialize(tue::serialization::InputArchive& m, ImageMask& mask);


//void serialize(const WorldModel& wm, tue::config::Writer& w);



// DESERIALIZATION



//void deserialize(tue::config::Reader& r, UpdateRequest& req);

}

#endif
