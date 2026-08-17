#include "ed/world_model.h"

#include "ed/entity.h"
#include "ed/measurement_convex_hull.h"
#include "ed/property.h"
#include "ed/relation.h"
#include "ed/time.h"
#include "ed/types.h"
#include "ed/update_request.h"

#include <algorithm>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <cstddef>
#include <geolib/datatypes.h>
#include <iostream>
#include <map>
#include <ostream>
#include <queue>
#include <set>
#include <string>
#include <tue/config/data_pointer.h>
#include <vector>

#include "ed/property_key_db.h"
#include "ed/uuid.h"

namespace ed
{

// --------------------------------------------------------------------------------

WorldModel::WorldModel(const PropertyKeyDB* prop_key_db) : property_info_db_(prop_key_db) {}

// --------------------------------------------------------------------------------

void WorldModel::update(const UpdateRequest& req)
{
    if (req.empty())
        return;

    // Increase revision number
    ++revision_;

    std::map<UUID, EntityPtr> new_entities;

    // Update associated measurements
    for (const auto& measurement : req.measurements)
    {
        EntityPtr const e = getOrAddEntity(measurement.first, new_entities);
        const std::vector<MeasurementConstPtr>& measurements = measurement.second;
        for (const auto& it2 : measurements)
        {
            e->addMeasurement(it2);
        }
    }

    // Update poses
    for (const auto& pose : req.poses)
    {
        EntityPtr const e = getOrAddEntity(pose.first, new_entities);
        e->setPose(pose.second);
    }

    for (const UUID& id : req.poses_removed)
    {
        EntityPtr const e = getOrAddEntity(id, new_entities);
        e->removePose();
    }

    // Update visuals
    for (const auto& visual : req.visuals)
    {
        EntityPtr const e = getOrAddEntity(visual.first, new_entities);
        e->setVisual(visual.second);

        Idx idx = 0;
        if (findEntityIdx(e->id(), idx))
        {
            entity_visual_revisions_[idx] = revision_;
        }
    }

    // Update collisions
    for (const auto& collision : req.collisions)
    {
        EntityPtr const e = getOrAddEntity(collision.first, new_entities);
        e->setCollision(collision.second);

        Idx idx = 0;
        if (findEntityIdx(e->id(), idx))
        {
            entity_collision_revisions_[idx] = revision_;
        }
    }

    // Update convex hulls new
    for (const auto& it : req.convex_hulls_new)
    {
        EntityPtr const e = getOrAddEntity(it.first, new_entities);
        for (const auto& it2 : it.second)
        {
            const ed::MeasurementConvexHull& m = it2.second;
            e->setConvexHull(m.convex_hull, m.pose, m.timestamp, it2.first);
        }

        Idx idx = 0;
        if (findEntityIdx(e->id(), idx))
        {
            entity_visual_revisions_[idx] = revision_;
            entity_collision_revisions_[idx] = revision_;
        }
    }

    // Update volumes
    for (const auto& it : req.volumes_removed)
    {
        EntityPtr const e = getOrAddEntity(it.first, new_entities);
        const std::set<std::string>& volume_names = it.second;
        for (const auto& volume_name : volume_names)
            e->removeVolume(volume_name);
        Idx idx = 0;
        if (findEntityIdx(e->id(), idx))
        {
            entity_volumes_revisions_[idx] = revision_;
        }
    }
    for (const auto& it : req.volumes_added)
    {
        EntityPtr const e = getOrAddEntity(it.first, new_entities);
        const std::map<std::string, geo::ShapeConstPtr>& volumes = it.second;
        for (const auto& volume : volumes)
        {
            e->addVolume(volume.first, volume.second);
        }
        Idx idx = 0;
        if (findEntityIdx(e->id(), idx))
        {
            entity_volumes_revisions_[idx] = revision_;
        }
    }

    // Update types
    for (const auto& type : req.types)
    {
        EntityPtr const e = getOrAddEntity(type.first, new_entities);
        e->setType(type.second);
    }

    for (const auto& it : req.type_sets_added)
    {
        EntityPtr const e = getOrAddEntity(it.first, new_entities);
        const std::set<std::string>& type_set = it.second;
        for (const auto& it2 : type_set)
            e->addType(it2);
    }

    for (const auto& it : req.type_sets_removed)
    {
        EntityPtr const e = getOrAddEntity(it.first, new_entities);
        const std::set<std::string>& type_set = it.second;
        for (const auto& it2 : type_set)
            e->removeType(it2);
    }

    // Update existence probabilities
    for (const auto& existence_probabilitie : req.existence_probabilities)
    {
        EntityPtr const e = getOrAddEntity(existence_probabilitie.first, new_entities);
        e->setExistenceProbability(existence_probabilitie.second);
    }

    // Update last update timestamps
    for (const auto& last_update_timestamp : req.last_update_timestamps)
    {
        EntityPtr const e = getOrAddEntity(last_update_timestamp.first, new_entities);
        e->setLastUpdateTimestamp(last_update_timestamp.second);
    }

    // Update relations
    for (const auto& relation : req.relations)
    {
        Idx idx1 = 0;
        if (findEntityIdx(relation.first, idx1))
        {
            const std::map<UUID, RelationConstPtr>& rels = relation.second;
            for (const auto& rel : rels)
            {
                Idx idx2 = 0;
                if (findEntityIdx(rel.first, idx2))
                    setRelation(idx1, idx2, rel.second);
                else
                    std::cout << "WorldModel::update (relation): unknown entity: '" << rel.first << "'." << '\n';
            }
        }
        else
            std::cout << "WorldModel::update (relation): unknown entity: '" << relation.first << "'." << '\n';
    }

    // Update flags
    for (const auto& added_flag : req.added_flags)
    {
        EntityPtr const e = getOrAddEntity(added_flag.first, new_entities);
        e->setFlag(added_flag.second);
    }

    for (const auto& removed_flag : req.removed_flags)
    {
        EntityPtr const e = getOrAddEntity(removed_flag.first, new_entities);
        e->removeFlag(removed_flag.second);
    }

    // Update additional info (data)
    for (const auto& data : req.datas)
    {
        EntityPtr const e = getOrAddEntity(data.first, new_entities);

        tue::config::DataPointer params;
        params.add(e->data());
        params.add(data.second);

        e->setData(params);
    }

    for (const auto& propertie : req.properties)
    {
        EntityPtr const e = getOrAddEntity(propertie.first, new_entities);
        const std::map<Idx, Property>& props = propertie.second;

        for (const auto& prop : props)
        {
            const Property& p = prop.second;
            e->setProperty(prop.first, p);
        }
    }

    // Remove entities
    for (const auto& removed_entitie : req.removed_entities)
    {
        removeEntity(removed_entitie);
    }
}

// --------------------------------------------------------------------------------

struct SearchNode
{
    SearchNode() = default;

    SearchNode(Idx parent_, Idx relation_, bool inverse_) : parent(parent_), relation(relation_), inverse(inverse_) {}

    Idx parent{};
    Idx relation{};
    bool inverse{};
};

// --------------------------------------------------------------------------------

bool WorldModel::calculateTransform(const UUID& source, const UUID& target, const Time& time, geo::Pose3D& tf) const
{
    Idx s = 0;
    Idx t = 0;
    if (!findEntityIdx(source, s) || !findEntityIdx(target, t))
        return false;

    std::queue<Idx> q;
    std::map<Idx, SearchNode> visited;

    q.push(s);
    visited[s] = SearchNode(INVALID_IDX, INVALID_IDX, true);

    while (!q.empty())
    {
        Idx const n = q.front();
        q.pop();

        if (n == t)
        {
            // Calculate transformation
            tf = geo::Pose3D::identity();

            Idx u = n;

            while (u != s)
            {
                auto const it = visited.find(u);
                const SearchNode& sn = it->second;

                const RelationConstPtr& r = relations_[sn.relation];

                geo::Pose3D tr;
                if (!r->calculateTransform(time, tr))
                {
                    std::cout << "WorldModel::calculateTransform: transform could not be calculated. THIS SHOULD NEVER "
                                 "HAPPEN!"
                              << '\n';
                    return false;
                }

                if (sn.inverse)
                    tf = tr.inverse() * tf;
                else
                    tf = tr * tf;

                u = sn.parent;
            }

            return true;
        }

        // Push all nodes that point to this node
        const std::map<Idx, Idx>& transforms_to = entities_[n]->relationsTo();
        for (auto it : transforms_to)
        {
            Idx const n2 = it.first;
            if (visited.find(n2) == visited.end())
            {
                visited[n2] = SearchNode(n, it.second, false);
                q.push(n2);
            }
        }

        // Push all nodes this node points to
        const std::map<Idx, Idx>& transforms_from = entities_[n]->relationsFrom();
        for (auto it : transforms_from)
        {
            Idx const n2 = it.first;
            if (visited.find(n2) == visited.end())
            {
                visited[n2] = SearchNode(n, it.second, true);
                q.push(n2);
            }
        }
    }

    return false;
}

// --------------------------------------------------------------------------------

void WorldModel::setRelation(Idx parent, Idx child, const RelationConstPtr& r)
{
    const EntityConstPtr& p = entities_[parent];
    const EntityConstPtr& c = entities_[child];

    if (!p || !c)
    {
        std::cout << "[ED] ERROR: Invalid relation addition: parent or child does not exit." << '\n';
        return;
    }

    Idx r_idx = p->relationTo(child);
    if (r_idx == INVALID_IDX)
    {
        r_idx = addRelation(r);

        EntityPtr const p_new(new Entity(*entities_[parent]));
        EntityPtr const c_new(new Entity(*entities_[child]));

        p_new->setRelationTo(child, r_idx);
        c_new->setRelationFrom(parent, r_idx);

        entities_[parent] = p_new;
        entities_[child] = c_new;
    }
    else
    {
        relations_[r_idx] = r;
    }

    // Update entity revisions
    for (std::size_t i = entity_revisions_.size(); i < std::max(parent, child) + 1; ++i)
        entity_revisions_.push_back(0);
    entity_revisions_[parent] = revision_;
    entity_revisions_[child] = revision_;
}

// --------------------------------------------------------------------------------

Idx WorldModel::addRelation(const RelationConstPtr& r)
{
    Idx const r_idx = relations_.size();
    relations_.push_back(r);
    return r_idx;
}

// --------------------------------------------------------------------------------

void WorldModel::setEntity(const UUID& id, const EntityConstPtr& e)
{
    auto const it_idx = entity_map_.find(id);
    if (it_idx == entity_map_.end())
    {
        addNewEntity(e);
    }
    else
    {
        entities_[it_idx->second] = e;
    }
}

// --------------------------------------------------------------------------------

void WorldModel::removeEntity(const UUID& id)
{
    auto const it_idx = entity_map_.find(id);
    if (it_idx != entity_map_.end())
    {
        entities_[it_idx->second].reset();
        entity_revisions_[it_idx->second] = revision_;
        entity_visual_revisions_[it_idx->second] = 0;
        entity_collision_revisions_[it_idx->second] = 0;
        entity_volumes_revisions_[it_idx->second] = 0;
        entity_empty_spots_.push(it_idx->second);
        entity_map_.erase(it_idx);
    }
}

// --------------------------------------------------------------------------------

EntityPtr WorldModel::getOrAddEntity(const UUID& id, std::map<UUID, EntityPtr>& new_entities)
{
    // Check if the id is already in the new_entities map. If so, return it
    auto const it_e = new_entities.find(id);
    if (it_e != new_entities.end())
        return it_e->second;

    EntityPtr e;

    Idx idx = 0;
    if (findEntityIdx(id, idx))
    {
        // Create a copy of the existing entity
        e = boost::make_shared<Entity>(*entities_[idx]);

        // Set the copy
        entities_[idx] = e;
    }
    else
    {
        // Does not yet exist
        e = boost::make_shared<Entity>(id);
        idx = addNewEntity(e);
    }

    // Update entity revision
    e->setRevision(revision_);

    new_entities[id] = e;

    for (std::size_t i = entity_revisions_.size(); i < idx + 1; ++i)
        entity_revisions_.push_back(0);
    entity_revisions_[idx] = revision_;

    return e;
}

// --------------------------------------------------------------------------------

bool WorldModel::findEntityIdx(const UUID& id, Idx& idx) const
{
    if (id.idx != INVALID_IDX && entities_[id.idx] && entities_[id.idx]->id() == id.str())
    {
        idx = id.idx;
        return true;
    }

    auto const it = entity_map_.find(id);
    if (it == entity_map_.end())
        return false;

    idx = it->second;
    id.idx = idx;
    return true;
}

// --------------------------------------------------------------------------------

Idx WorldModel::addNewEntity(const EntityConstPtr& e)
{
    Idx idx = 0;
    if (entity_empty_spots_.empty())
    {
        idx = entities_.size();
        entity_map_[e->id()] = idx;
        entities_.push_back(e);
        entity_visual_revisions_.push_back(0);
        entity_collision_revisions_.push_back(0);
        entity_volumes_revisions_.push_back(0);
    }
    else
    {
        idx = entity_empty_spots_.front();
        entity_empty_spots_.pop();
        entity_map_[e->id()] = idx;
        entities_[idx] = e;
    }

    return idx;
}

// --------------------------------------------------------------------------------

const PropertyKeyDBEntry* WorldModel::getPropertyInfo(const std::string& name) const
{
    if (!property_info_db_)
        return nullptr;

    return property_info_db_->getPropertyKeyDBEntry(name);
}

} // namespace ed
