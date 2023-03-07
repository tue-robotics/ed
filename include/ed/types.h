#ifndef ED_TYPES_H_
#define ED_TYPES_H_

//#include <rgbd/types.h>
//#include <geolib/datatypes.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <limits>
#include <stdint.h>

namespace tf2_ros {

class Buffer;

}

namespace ed
{

typedef uint64_t Idx;
static const Idx INVALID_IDX = std::numeric_limits<Idx>::max();

// For easy switching to std pointers
using boost::shared_ptr;
using boost::make_shared;
using boost::const_pointer_cast;
using boost::dynamic_pointer_cast;
using boost::static_pointer_cast;

class Measurement;
typedef shared_ptr<Measurement> MeasurementPtr;
typedef shared_ptr<const Measurement> MeasurementConstPtr;

class Entity;
typedef shared_ptr<Entity> EntityPtr;
typedef shared_ptr<const Entity> EntityConstPtr;

class Plugin;
typedef shared_ptr<Plugin> PluginPtr;
typedef shared_ptr<const Plugin> PluginConstPtr;

class WorldModel;
typedef shared_ptr<WorldModel> WorldModelPtr;
typedef shared_ptr<const WorldModel> WorldModelConstPtr;

class UpdateRequest;
typedef shared_ptr<UpdateRequest> UpdateRequestPtr;
typedef shared_ptr<const UpdateRequest> UpdateRequestConstPtr;

class PluginContainer;
typedef shared_ptr<PluginContainer> PluginContainerPtr;
typedef shared_ptr<const PluginContainer> PluginContainerConstPtr;

class SensorModule;
typedef shared_ptr<SensorModule> SensorModulePtr;
typedef shared_ptr<const SensorModule> SensorModuleConstPtr;

class RGBDALModule;
typedef shared_ptr<RGBDALModule> RGBDALModulePtr;
typedef shared_ptr<const RGBDALModule> RGBDALModuleConstPtr;

class RGBDSegModule;
typedef shared_ptr<RGBDSegModule> RGBDSegModulePtr;
typedef shared_ptr<const RGBDSegModule> RGBDSegModuleConstPtr;

class PerceptionModule;
typedef shared_ptr<PerceptionModule> PerceptionModulePtr;
typedef shared_ptr<const PerceptionModule> PerceptionModuleConstPtr;

class Relation;
typedef shared_ptr<Relation> RelationPtr;
typedef shared_ptr<const Relation> RelationConstPtr;

class ConvexHull2D;
class ImageMask;

class UUID;

typedef std::string TYPE;

// tf2_ros::Buffer
typedef shared_ptr<tf2_ros::Buffer> TFBufferPtr;
typedef shared_ptr<const tf2_ros::Buffer> TFBufferConstPtr;

}

#endif
