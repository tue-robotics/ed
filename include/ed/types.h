#ifndef ED_TYPES_H_
#define ED_TYPES_H_

//#include <rgbd/types.h>
//#include <geolib/datatypes.h>

#include <boost/shared_ptr.hpp>
#include <limits>
#include <stdint.h>

namespace ed
{

typedef uint64_t Idx;
static const Idx INVALID_IDX = std::numeric_limits<Idx>::max();

class Measurement;
typedef boost::shared_ptr<Measurement> MeasurementPtr;
typedef boost::shared_ptr<const Measurement> MeasurementConstPtr;

class Entity;
typedef boost::shared_ptr<Entity> EntityPtr;
typedef boost::shared_ptr<const Entity> EntityConstPtr;

class Plugin;
typedef boost::shared_ptr<Plugin> PluginPtr;
typedef boost::shared_ptr<const Plugin> PluginConstPtr;

class WorldModel;
typedef boost::shared_ptr<WorldModel> WorldModelPtr;
typedef boost::shared_ptr<const WorldModel> WorldModelConstPtr;

class UpdateRequest;
typedef boost::shared_ptr<UpdateRequest> UpdateRequestPtr;
typedef boost::shared_ptr<const UpdateRequest> UpdateRequestConstPtr;

class PluginContainer;
typedef boost::shared_ptr<PluginContainer> PluginContainerPtr;
typedef boost::shared_ptr<const PluginContainer> PluginContainerConstPtr;

class SensorModule;
typedef boost::shared_ptr<SensorModule> SensorModulePtr;
typedef boost::shared_ptr<const SensorModule> SensorModuleConstPtr;

class RGBDALModule;
typedef boost::shared_ptr<RGBDALModule> RGBDALModulePtr;
typedef boost::shared_ptr<const RGBDALModule> RGBDALModuleConstPtr;

class RGBDSegModule;
typedef boost::shared_ptr<RGBDSegModule> RGBDSegModulePtr;
typedef boost::shared_ptr<const RGBDSegModule> RGBDSegModuleConstPtr;

class PerceptionModule;
typedef boost::shared_ptr<PerceptionModule> PerceptionModulePtr;
typedef boost::shared_ptr<const PerceptionModule> PerceptionModuleConstPtr;

class Relation;
typedef boost::shared_ptr<Relation> RelationPtr;
typedef boost::shared_ptr<const Relation> RelationConstPtr;

class ConvexHull2D;
class ImageMask;

class UUID;

typedef std::string TYPE;

}

#endif
