#ifndef ED_TYPES_H_
#define ED_TYPES_H_

// #include <rgbd/types.h>
// #include <geolib/datatypes.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <limits>
#include <stdint.h>

namespace tf2_ros { class Buffer; }

namespace ed
{

using Idx = uint64_t;
static const Idx INVALID_IDX = std::numeric_limits<Idx>::max();

// For easy switching to std pointers
using boost::const_pointer_cast;
using boost::dynamic_pointer_cast;
using boost::make_shared;
using boost::shared_ptr;
using boost::static_pointer_cast;

class Measurement;
using MeasurementPtr = shared_ptr<Measurement>;
using MeasurementConstPtr = shared_ptr<const Measurement>;

class Entity;
using EntityPtr = shared_ptr<Entity>;
using EntityConstPtr = shared_ptr<const Entity>;

class Plugin;
using PluginPtr = shared_ptr<Plugin>;
using PluginConstPtr = shared_ptr<const Plugin>;

class WorldModel;
using WorldModelPtr = shared_ptr<WorldModel>;
using WorldModelConstPtr = shared_ptr<const WorldModel>;

class UpdateRequest;
using UpdateRequestPtr = shared_ptr<UpdateRequest>;
using UpdateRequestConstPtr = shared_ptr<const UpdateRequest>;

class PluginContainer;
using PluginContainerPtr = shared_ptr<PluginContainer>;
using PluginContainerConstPtr = shared_ptr<const PluginContainer>;

class SensorModule;
using SensorModulePtr = shared_ptr<SensorModule>;
using SensorModuleConstPtr = shared_ptr<const SensorModule>;

class RGBDALModule;
using RGBDALModulePtr = shared_ptr<RGBDALModule>;
using RGBDALModuleConstPtr = shared_ptr<const RGBDALModule>;

class RGBDSegModule;
using RGBDSegModulePtr = shared_ptr<RGBDSegModule>;
using RGBDSegModuleConstPtr = shared_ptr<const RGBDSegModule>;

class PerceptionModule;
using PerceptionModulePtr = shared_ptr<PerceptionModule>;
using PerceptionModuleConstPtr = shared_ptr<const PerceptionModule>;

class Relation;
using RelationPtr = shared_ptr<Relation>;
using RelationConstPtr = shared_ptr<const Relation>;

struct ConvexHull2D;
class ImageMask;

class UUID;

using TYPE = std::string;

// tf2_ros::Buffer
using TFBufferPtr = shared_ptr<tf2_ros::Buffer>;
using TFBufferConstPtr = shared_ptr<const tf2_ros::Buffer>;

} // namespace ed

#endif
