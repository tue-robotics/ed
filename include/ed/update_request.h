#ifndef ED_UPDATE_REQUEST_H_
#define ED_UPDATE_REQUEST_H_

#include "ed/types.h"
#include "ed/uuid.h"
#include "ed/property.h"
#include "ed/property_key.h"
#include "ed/property_key_db.h"

#include <tue/config/data_pointer.h>

#include <map>
#include <set>
#include <vector>
#include <geolib/datatypes.h>

#include "ed/convex_hull_2d.h"
#include "ed/convex_hull.h"
#include "ed/measurement_convex_hull.h"

namespace ed
{

class UpdateRequest
{

public:

    UpdateRequest() : is_sync_update(false) {}


    // MEASUREMENTS

    std::map<UUID, std::vector<MeasurementConstPtr> > measurements;
    void addMeasurement(const UUID& id, const MeasurementConstPtr& m) { measurements[id].push_back(m); flagUpdated(id); }

    void addMeasurements(const UUID& id, const std::vector<MeasurementConstPtr>& measurements_)
    {
        if (measurements_.empty())
            return;

        std::vector<MeasurementConstPtr>& v = measurements[id];
        v.insert(v.end(), measurements_.begin(), measurements_.end());

        flagUpdated(id);
    }


    // VISUALS
    std::map<UUID, geo::ShapeConstPtr> visuals;
    [[deprecated("Use setVisual() instead.")]]
    void setShape(const UUID& id, const geo::ShapeConstPtr& shape) { setVisual(id, shape); }
    void setVisual(const UUID& id, const geo::ShapeConstPtr& visual) { visuals[id] = visual; flagUpdated(id); }

    // COLLISIONS
    std::map<UUID, geo::ShapeConstPtr> collisions;
    void setCollision(const UUID& id, const geo::ShapeConstPtr& collision) { collisions[id] = collision; flagUpdated(id); }

    // VOLUMES
    std::map<UUID, std::map<std::string, geo::ShapeConstPtr> > volumes_added;
    void addVolume(const UUID& id, const std::string Volume_name, const geo::ShapeConstPtr& Volume_shape)
    {   std::map<UUID, std::map<std::string, geo::ShapeConstPtr> >::iterator it = volumes_added.find(id);
        if (it != volumes_added.end())
        {
            std::map<std::string, geo::ShapeConstPtr>::iterator it2 = it->second.find(Volume_name);
            if (it2 != it->second.end())
                it2->second = Volume_shape;
            else
                it->second[Volume_name] = Volume_shape;
        }
        else
        {
            std::map<std::string, geo::ShapeConstPtr> volume_map;
            volume_map[Volume_name] = Volume_shape;
            volumes_added[id] = volume_map;
        }
        flagUpdated(id);
    }

    std::map<UUID, std::set<std::string> > volumes_removed;
    void removeVolume(const UUID& id, const std::string Volume_name) { volumes_removed[id].insert(Volume_name); flagUpdated(id); }


    // CONVEX HULLS NEW

    std::map<UUID, std::map<std::string, ed::MeasurementConvexHull> > convex_hulls_new;
    void setConvexHullNew(const UUID& id, const ed::ConvexHull& convex_hull, const geo::Pose3D& pose, double time, std::string source = "")
    {
        ed::MeasurementConvexHull& m = convex_hulls_new[id][source];
        m.convex_hull = convex_hull;
        m.pose = pose;
        m.timestamp = time;
        flagUpdated(id);
    }

    void removeConvexHullNew(const UUID& id, const std::string& source)
    {
        // For now, signal that the convex hull must be removed by setting an empty chull
        convex_hulls_new[id][source] = ed::MeasurementConvexHull();
    }


    // TYPES

    std::map<UUID, std::string> types;
    void setType(const UUID& id, const std::string& type) { types[id] = type; flagUpdated(id); }

    std::map<UUID, std::set<std::string> > type_sets_added;
    void addType(const UUID& id, const std::string& type) { type_sets_added[id].insert(type); flagUpdated(id); }

    std::map<UUID, std::set<std::string> > type_sets_removed;
    void removeType(const UUID& id, const std::string& type) { type_sets_removed[id].insert(type); flagUpdated(id); }


    // PROBABILITY OF EXISTENCE

    std::map<UUID, double> existence_probabilities;

    void setExistenceProbability(const UUID& id, double prob) { existence_probabilities[id] = prob; }


    // LAST UPDATE TIMESTAMP

    std::map<UUID, double> last_update_timestamps;

    void setLastUpdateTimestamp(const UUID& id, double t) { last_update_timestamps[id] = t; }


    // POSES

    std::map<UUID, geo::Pose3D> poses;
    void setPose(const UUID& id, const geo::Pose3D& pose) { poses[id] = pose; flagUpdated(id); }


    // RELATIONS

    std::map<UUID, std::map<UUID, RelationConstPtr> > relations;
    void setRelation(const UUID& id1, const UUID& id2, const RelationConstPtr& r) { relations[id1][id2] = r; flagUpdated(id1); flagUpdated(id2);}


    // DATA

    std::map<UUID, tue::config::DataConstPointer> datas;

    void addData(const UUID& id, const tue::config::DataConstPointer& data)
    {
        std::map<UUID, tue::config::DataConstPointer>::iterator it = datas.find(id);
        if (it == datas.end())
        {
            datas[id] = data;
        }
        else
        {
            tue::config::DataPointer data_total;
            data_total.add(it->second);
            data_total.add(data);

            it->second = data_total;
        }

        flagUpdated(id);
    }

    std::map<UUID, std::map<Idx, Property> > properties;

    template<typename T>
    void setProperty(const UUID& id, const PropertyKey<T>& key, const T& value)
    {
        if (!key.valid())
            return;

        Property& p = properties[id][key.idx];
        p.entry = key.entry;
        p.value = value;
        flagUpdated(id);
    }

    void setProperty(const UUID& id, const PropertyKeyDBEntry* entry, const ed::Variant& v)
    {
        Property& p = properties[id][entry->idx];
        p.entry = entry;
        p.value = v;
        flagUpdated(id);
    }


    // REMOVED ENTITIES

    std::set<UUID> removed_entities;

    void removeEntity(const UUID& id) { removed_entities.insert(id); flagUpdated(id); }


    // FLAGS

    std::map<ed::UUID, std::string> added_flags;

    void setFlag(const UUID& id, const std::string& flag) { added_flags[id] = flag; flagUpdated(id); }

    std::map<ed::UUID, std::string> removed_flags;

    void removeFlag(const UUID& id, const std::string& flag) { removed_flags[id] = flag; flagUpdated(id); }



    // UPDATED (AND REMOVED) ENTITIES

    std::set<UUID> updated_entities;

    bool empty() const { return updated_entities.empty(); }


    // Is true if the update was created for synchronization only (used by ed_cloud)

    bool is_sync_update;

    void setSyncUpdate(bool b = true) { is_sync_update = b; }


private:

    void flagUpdated(const ed::UUID& id) { updated_entities.insert(id); }

};

}

#endif
