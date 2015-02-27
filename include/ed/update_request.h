#ifndef ED_UPDATE_REQUEST_H_
#define ED_UPDATE_REQUEST_H_

#include "ed/types.h"
#include "ed/uuid.h"

#include <tue/config/data_pointer.h>

#include <map>
#include <vector>
#include <geolib/datatypes.h>

#include <ed/convex_hull_2d.h>

namespace ed
{

class UpdateRequest
{

public:

    // MEASUREMENTS

    std::map<UUID, std::vector<MeasurementConstPtr> > measurements;
    void addMeasurement(const UUID& id, const MeasurementConstPtr& m) { measurements[id].push_back(m); }

    void addMeasurements(const UUID& id, const std::vector<MeasurementConstPtr>& measurements_)
    {
        std::vector<MeasurementConstPtr>& v = measurements[id];
        v.insert(v.end(), measurements_.begin(), measurements_.end());
    }


    // SHAPES

    std::map<UUID, geo::ShapeConstPtr> shapes;
    void setShape(const UUID& id, const geo::ShapeConstPtr& shape) { shapes[id] = shape; }


    // CONVEX HULLS

    std::map<UUID, ed::ConvexHull2D> convex_hulls;
    void setConvexHull(const UUID& id, const ed::ConvexHull2D& convex_hull) { convex_hulls[id] = convex_hull; }

    // TYPES

    std::map<UUID, std::string> types;
    void setType(const UUID& id, const std::string& type) { types[id] = type; }


    // POSES

    std::map<UUID, geo::Pose3D> poses;
    void setPose(const UUID& id, const geo::Pose3D& pose) { poses[id] = pose; }


    // RELATIONS

    std::map<UUID, std::map<UUID, RelationConstPtr> > relations;
    void setRelation(const UUID& id1, const UUID& id2, const RelationConstPtr& r) { relations[id1][id2] = r; }


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
    }


    // REMOVED ENTITIES

    std::set<UUID> removed_entities;

    void removeEntity(const UUID& id) { removed_entities.insert(id); }



    bool empty() const
    {
        return measurements.empty() &&
               shapes.empty() &&
               types.empty() &&
               poses.empty() &&
               relations.empty() &&
               removed_entities.empty() &&
               datas.empty();
    }

};

}

#endif
