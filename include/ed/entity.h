#ifndef ED_ENTITY_H_
#define ED_ENTITY_H_

#include "ed/convex_hull.h"
#include "ed/convex_hull_2d.h"
#include "ed/types.h"
#include "ed/uuid.h"
#include <cstdint>

#include <tue/config/data.h>

#include <algorithm>
#include <boost/circular_buffer.hpp>

#include "ed/property.h"
#include "ed/property_key.h"

#include "ed/logging.h"

#include "ed/measurement_convex_hull.h"

#include <map>
#include <set>
#include <vector>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

class Entity
{

public:
    explicit Entity(UUID id = generateID(), TYPE type = "", const unsigned int& measurement_buffer_size = 5);
    ~Entity();

    static UUID generateID();
    const UUID& id() const { return id_; }

    const TYPE& type() const { return type_; }
    void setType(const TYPE& type)
    {
        type_ = type;
        types_.insert(type);
    }

    const std::set<TYPE>& types() const { return types_; }
    void addType(const TYPE& type) { types_.insert(type); }
    void removeType(const TYPE& type) { types_.erase(type); }
    bool hasType(const TYPE& type) const { return types_.contains(type); }

    void measurements(std::vector<MeasurementConstPtr>& measurements, double min_timestamp = 0) const;
    void measurements(std::vector<MeasurementConstPtr>& measurements, unsigned int num) const;
    MeasurementConstPtr lastMeasurement() const;
    unsigned int measurementSeq() const { return measurements_seq_; }
    MeasurementConstPtr bestMeasurement() const { return best_measurement_; }

    void addMeasurement(const MeasurementConstPtr& measurement);

    [[deprecated("Use visual() or collision() instead.")]]
    geo::ShapeConstPtr shape() const
    {
        return visual();
    }
    geo::ShapeConstPtr visual() const { return visual_; }
    geo::ShapeConstPtr collision() const { return collision_; }
    [[deprecated("Use setVisual() or setCollision() instead.")]]
    void setShape(const geo::ShapeConstPtr& shape)
    {
        setVisual(shape);
    }
    void setVisual(const geo::ShapeConstPtr& visual);
    void setCollision(const geo::ShapeConstPtr& collision);

    const std::map<std::string, geo::ShapeConstPtr>& volumes() const { return volumes_; }
    void addVolume(const std::string& volume_name, const geo::ShapeConstPtr& volume_shape)
    {
        volumes_[volume_name] = volume_shape;
        ++volumes_revision_;
    }
    void removeVolume(const std::string& volume_name)
    {
        volumes_.erase(volume_name);
        ++volumes_revision_;
    }

    [[deprecated("Use visualRevision(), collisionRevision() or volumesRevision() instead.")]]
    std::uint64_t shapeRevision() const
    {
        return visualRevision();
    }
    std::uint64_t visualRevision() const { return visual_ ? visual_revision_ : 0; }
    std::uint64_t collisionRevision() const { return collision_ ? collision_revision_ : 0; }
    std::uint64_t volumesRevision() const { return !volumes_.empty() ? volumes_revision_ : 0; }

    const ConvexHull& convexHull() const { return convex_hull_new_; }

    void setConvexHull(const ConvexHull& convex_hull,
                       const geo::Pose3D& pose,
                       double time,
                       const std::string& source = "")
    {
        if (convex_hull.points.empty())
        {
            // This signals that the measurement convex hull must be removed
            convex_hull_map_.erase(source);
        }
        else
        {
            ed::MeasurementConvexHull& m = convex_hull_map_[source];
            m.convex_hull = convex_hull;
            m.pose = pose;
            m.timestamp = time;
        }

        updateConvexHull();
    }

    const std::map<std::string, MeasurementConvexHull>& convexHullMap() const { return convex_hull_map_; }

    const geo::Pose3D& pose() const
    {
        if (!has_pose_)
            log::warning() << "Someone's accessing an entity's pose while it doesnt have one." << '\n';
        return pose_;
    }

    void setPose(const geo::Pose3D& pose)
    {
        pose_ = pose;
        if (visual_)
            updateConvexHullFromVisual();

        has_pose_ = true;
    }

    void removePose() { has_pose_ = false; }

    bool hasPose() const { return has_pose_; }

    const tue::config::DataConstPointer& data() const { return config_; }
    void setData(const tue::config::DataConstPointer& data) { config_ = data; }

    //    inline double creationTime() const { return creation_time_; }

    void setRelationTo(Idx child_idx, Idx r_idx) { relations_to_[child_idx] = r_idx; }

    void setRelationFrom(Idx parent_idx, Idx r_idx) { relations_from_[parent_idx] = r_idx; }

    Idx relationTo(Idx child_idx) const
    {
        auto const it = relations_to_.find(child_idx);
        if (it == relations_to_.end())
            return INVALID_IDX;
        return it->second;
    }

    Idx relationFrom(Idx parent_idx) const
    {
        auto const it = relations_from_.find(parent_idx);
        if (it == relations_from_.end())
            return INVALID_IDX;
        return it->second;
    }

    const std::map<Idx, Idx>& relationsFrom() const { return relations_from_; }

    const std::map<Idx, Idx>& relationsTo() const { return relations_to_; }

    template <typename T> const T* property(const PropertyKey<T>& key) const
    {
        auto const it = properties_.find(key.idx);
        if (it == properties_.end())
            return nullptr;

        const Property& p = it->second;

        try
        {
            return &p.value.getValue<T>();
        }
        catch (std::bad_cast& e)
        {
            return nullptr;
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
        auto const it = properties_.find(idx);
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

        revision_ = std::max(revision_, p.revision);
    }

    const std::map<Idx, Property>& properties() const { return properties_; }

    std::uint64_t revision() const { return revision_; }

    void setRevision(std::uint64_t revision) { revision_ = revision; }

    void setExistenceProbability(double prob) { existence_prob_ = prob; }

    double existenceProbability() const { return existence_prob_; }

    void setLastUpdateTimestamp(double t) { last_update_timestamp_ = t; }

    double lastUpdateTimestamp() const { return last_update_timestamp_; }

    void setFlag(const std::string& flag) { flags_.insert(flag); }

    void removeFlag(const std::string& flag) { flags_.erase(flag); }

    bool hasFlag(const std::string& flag) const { return flags_.contains(flag); }

    const std::set<std::string>& flags() const { return flags_; }

private:
    UUID id_;

    std::uint64_t revision_{0};

    TYPE type_;

    std::set<TYPE> types_;

    double existence_prob_{1.0};

    double last_update_timestamp_{0};

    boost::circular_buffer<MeasurementConstPtr> measurements_;
    MeasurementConstPtr best_measurement_;
    unsigned int measurements_seq_{0};

    geo::ShapeConstPtr visual_;
    geo::ShapeConstPtr collision_;
    std::map<std::string, geo::ShapeConstPtr> volumes_;
    std::uint64_t visual_revision_{0};
    std::uint64_t collision_revision_{0};
    std::uint64_t volumes_revision_{0};

    std::map<std::string, MeasurementConvexHull> convex_hull_map_;
    ConvexHull convex_hull_new_;

    bool has_pose_{false};
    geo::Pose3D pose_;

    //    double creation_time_;

    tue::config::DataConstPointer config_;

    std::map<Idx, Idx> relations_from_;
    std::map<Idx, Idx> relations_to_;

    // Generic property map
    std::map<Idx, Property> properties_;

    void updateConvexHull();

    void updateConvexHullFromVisual();

    std::set<std::string> flags_;
};

} // namespace ed

#endif
