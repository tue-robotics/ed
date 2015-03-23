#ifndef entity_h_
#define entity_h_

#include "ed/types.h"
#include "ed/convex_hull_2d.h"
#include "ed/uuid.h"

#include <tue/config/data.h>

#include <boost/circular_buffer.hpp>
#include <ros/time.h>

#include "ed/property.h"
#include "ed/property_key.h"

namespace ed
{

struct GlobalData;

class Entity
{

public:
    Entity(const UUID& id, const GlobalDataConstPtr& global_data);
    ~Entity();

    static UUID generateID();
    const UUID& id() const { return id_; }

    const TYPE& type() const { return type_; }
    void setType(const TYPE& type) { type_ = type; }

    void measurements(std::vector<MeasurementConstPtr>& measurements, double min_timestamp = 0) const;
    void measurements(std::vector<MeasurementConstPtr>& measurements, unsigned int num) const;
    MeasurementConstPtr lastMeasurement() const;
    unsigned int measurementSeq() const { return measurements_seq_; }
    MeasurementConstPtr bestMeasurement() const { return best_measurement_; }

    void addMeasurement(MeasurementConstPtr measurement);

    inline geo::ShapeConstPtr shape() const { return shape_; }
    void setShape(const geo::ShapeConstPtr& shape);

    inline int shapeRevision() const{ return shape_ ? shape_revision_ : 0; }

    inline const ConvexHull2D& convexHull() const { return convex_hull_; }

    void setConvexHull(const ConvexHull2D& convex_hull) { convex_hull_ = convex_hull; }

    const geo::Pose3D& pose() const;

    void setPose(const geo::Pose3D& pose);

    inline const geo::Pose3D& velocity() const { return velocity_; }
    inline void setVelocity(const geo::Pose3D& velocity) { velocity_ = velocity; }

//    inline void setConfig(const tue::Configuration& config) { config_ = config; }
//    inline tue::Configuration getConfig() const { return config_.limitScope(); }

    inline const tue::config::DataConstPointer& data() const { return config_; }
    inline void setData(const tue::config::DataConstPointer& data) { config_ = data; }

    //! For debugging purposes
    bool in_frustrum;
    bool object_in_front;

//    inline double creationTime() const { return creation_time_; }

    inline void setRelationTo(Idx child_idx, Idx r_idx) { relations_to_[child_idx] = r_idx; }

    inline void setRelationFrom(Idx parent_idx, Idx r_idx) { relations_from_[parent_idx] = r_idx; }

    inline Idx relationTo(Idx child_idx) const
    {
        std::map<Idx, Idx>::const_iterator it = relations_to_.find(child_idx);
        if (it == relations_to_.end())
            return INVALID_IDX;
        return it->second;
    }

    inline Idx relationFrom(Idx parent_idx) const
    {
        std::map<Idx, Idx>::const_iterator it = relations_from_.find(parent_idx);
        if (it == relations_from_.end())
            return INVALID_IDX;
        return it->second;
    }

    const std::map<Idx, Idx>& relationsFrom() const { return relations_from_; }

    const std::map<Idx, Idx>& relationsTo() const { return relations_to_; }

    template<typename T>
    const T* property(const PropertyKey<T>& key) const
    {
        std::map<Idx, Property>::const_iterator it = properties_.find(key.idx);
        if (it == properties_.end())
            return 0;

        const Property& p = it->second;

        try
        {
            return &p.value.getValue<T>();
        }
        catch (std::bad_cast& e)
        {
            return 0;
        }
    }

//    template<typename T>
//    void setProperty(const PropertyKey<T>& key, const T& t)
//    {
//        if (!key.valid())
//            return;

//        std::map<Idx, Property>::iterator it = properties_.find(key.idx);
//        if (it == properties_.end())
//        {
//            Property& p = properties_[key.idx];
//            p.entry = key.entry;
//            p.revision = 0;
//            p.value.setValue(t);
//        }
//        else
//        {
//            Property& p = it->second;
//            p.value.setValue(t);
//            ++(p.revision);
//        }
//    }

    void setProperty(Idx idx, const Property& p)
    {
        std::map<Idx, Property>::iterator it = properties_.find(idx);
        if (it == properties_.end())
        {
            Property& p_new = properties_[idx];
            p_new.entry = p.entry;
            p_new.revision = p.revision;
            p_new.value = p.value;
        }
        else
        {
            Property& p_new = it->second;
            p_new.value = p.value;
            p_new.revision = p.revision;
        }

        if (revision_ < p.revision)
            revision_ = p.revision;
    }

    const std::map<Idx, Property>& properties() const { return properties_; }

    unsigned long revision() const { return revision_; }

    void setRevision(unsigned long revision) { revision_ = revision; }

private:

    UUID id_;

    unsigned long revision_;

    TYPE type_;

    boost::circular_buffer<MeasurementConstPtr> measurements_;
    MeasurementConstPtr best_measurement_;
    unsigned int measurements_seq_;

    boost::circular_buffer<std::pair<ConvexHull2D, double> > convex_hull_buffer_;

    geo::ShapeConstPtr shape_;
    int shape_revision_;
    ConvexHull2D convex_hull_;

    geo::Pose3D pose_;
    geo::Pose3D velocity_;
    geo::Vector3 average_displacement_vector_;

    void updateEntityState(MeasurementConstPtr m);
    void calculateVelocity();

//    double creation_time_;

    tue::config::DataConstPointer config_;

    std::map<Idx, Idx> relations_from_;
    std::map<Idx, Idx> relations_to_;

    // Generic property map
    std::map<Idx, Property> properties_;

    void updateConvexHull();

    GlobalDataConstPtr global_data_;

};

}

#endif
