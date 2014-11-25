#ifndef ED_UPDATE_REQUEST_H_
#define ED_UPDATE_REQUEST_H_

#include "ed/types.h"

#include <tue/config/data_pointer.h>

namespace ed
{

class UpdateRequest
{

public:

    // MEASUREMENTS

    std::map<UUID, std::vector<MeasurementConstPtr> > measurements;

    void addMeasurement(const UUID& id, const MeasurementConstPtr& m)
    {
        measurements[id].push_back(m);
    }

    void addMeasurements(const UUID& id, const std::vector<MeasurementConstPtr>& measurements_)
    {
        std::vector<MeasurementConstPtr>& v = measurements[id];
        v.insert(v.end(), measurements_.begin(), measurements_.end());
    }


    // DATA

    std::map<UUID, tue::config::DataConstPointer> datas;

    void addData(const UUID& id, const tue::config::DataConstPointer& data)
    {
        datas[id] = data;
    }


    // REMOVED ENTITIES

    std::set<UUID> removed_entities;

    void removeEntity(const UUID& id) { removed_entities.insert(id); }



    bool empty() const
    {
        return measurements.empty() &&
               removed_entities.empty() &&
               datas.empty();
    }

};

}

#endif
