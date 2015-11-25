#include "ed/world_model.h"

#include "ed/update_request.h"
#include "ed/entity.h"
#include "ed/relation.h"

#include <tue/config/reader.h>
#include <boost/make_shared.hpp>

#include "ed/property_key_db.h"

namespace ed
{

// --------------------------------------------------------------------------------

WorldModel::WorldModel(const PropertyKeyDB* prop_key_db) : revision_(0), property_info_db_(prop_key_db)
{
}

// --------------------------------------------------------------------------------

void WorldModel::update(const UpdateRequest& req)
{
    if (req.empty())
        return;

    // Increase revision number
    ++revision_;

    std::map<UUID, EntityPtr> new_entities;

    // Update associated measurements
    for(std::map<UUID, std::vector<MeasurementConstPtr> >::const_iterator it = req.measurements.begin(); it != req.measurements.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        const std::vector<MeasurementConstPtr>& measurements = it->second;
        for(std::vector<MeasurementConstPtr>::const_iterator it2 = measurements.begin(); it2 != measurements.end(); ++it2)
        {
            e->addMeasurement(*it2);
        }
    }

    // Update poses
    for(std::map<UUID, geo::Pose3D>::const_iterator it = req.poses.begin(); it != req.poses.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setPose(it->second);
    }

    // Update shapes
    for(std::map<UUID, geo::ShapeConstPtr>::const_iterator it = req.shapes.begin(); it != req.shapes.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setShape(it->second);

        Idx idx;
        if (findEntityIdx(e->id(), idx))
        {
            entity_shape_revisions_[idx] = revision_;
        }
    }

    // Update convex hulls new
    for(std::map<UUID, std::map<std::string, ed::MeasurementConvexHull> >::const_iterator it = req.convex_hulls_new.begin(); it != req.convex_hulls_new.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        for(std::map<std::string, ed::MeasurementConvexHull>::const_iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
        {
            const ed::MeasurementConvexHull& m = it2->second;
            e->setConvexHull(m.convex_hull, m.pose, m.timestamp, it2->first);
        }

        Idx idx;
        if (findEntityIdx(e->id(), idx))
        {
            entity_shape_revisions_[idx] = revision_;
        }
    }

    // Update types
    for(std::map<UUID, std::string>::const_iterator it = req.types.begin(); it != req.types.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setType(it->second);
    }

    for(std::map<UUID, std::set<std::string> >::const_iterator it = req.type_sets_.begin(); it != req.type_sets_.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        const std::set<std::string>& type_set = it->second;
        for(std::set<std::string>::const_iterator it2 = type_set.begin(); it2 != type_set.end(); ++it2)
            e->addType(*it2);
    }

    // Update existence probabilities
    for(std::map<UUID, double>::const_iterator it = req.existence_probabilities.begin(); it != req.existence_probabilities.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setExistenceProbability(it->second);
    }

    // Update last update timestamps
    for(std::map<UUID, double>::const_iterator it = req.last_update_timestamps.begin(); it != req.last_update_timestamps.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setLastUpdateTimestamp(it->second);
    }

    // Update relations
    for(std::map<UUID, std::map<UUID, RelationConstPtr> >::const_iterator it = req.relations.begin(); it != req.relations.end(); ++it)
    {
        Idx idx1;
        if (findEntityIdx(it->first, idx1))
        {
            const std::map<UUID, RelationConstPtr>& rels = it->second;
            for(std::map<UUID, RelationConstPtr>::const_iterator it2 = rels.begin(); it2 != rels.end(); ++it2)
            {
                Idx idx2;
                if (findEntityIdx(it2->first, idx2))
                    setRelation(idx1, idx2, it2->second);
                else
                    std::cout << "WorldModel::update (relation): unknown entity: '" << it2->first << "'." << std::endl;
            }
        }
        else
            std::cout << "WorldModel::update (relation): unknown entity: '" << it->first << "'." << std::endl;
    }

    // Update flags
    for(std::map<UUID, std::string>::const_iterator it = req.added_flags.begin(); it != req.added_flags.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setFlag(it->second);
    }

    for(std::map<UUID, std::string>::const_iterator it = req.removed_flags.begin(); it != req.removed_flags.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->removeFlag(it->second);
    }

    // Update additional info (data)
    for(std::map<UUID, tue::config::DataConstPointer>::const_iterator it = req.datas.begin(); it != req.datas.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);

        tue::config::DataPointer params;
        params.add(e->data());
        params.add(it->second);

        tue::config::Reader r(params);
        std::string type;
        if (r.value("type", type, tue::config::OPTIONAL))
            e->setType(type);

        e->setData(params);
    }

    for(std::map<UUID, std::map<Idx, Property> >::const_iterator it = req.properties.begin(); it != req.properties.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        const std::map<Idx, Property>& props = it->second;

        for(std::map<Idx, Property>::const_iterator it2 = props.begin(); it2 != props.end(); ++it2)
        {
            const Property& p = it2->second;
            e->setProperty(it2->first, p);
        }
    }

    // Remove entities
    for(std::set<UUID>::const_iterator it = req.removed_entities.begin(); it != req.removed_entities.end(); ++it)
    {
        removeEntity(*it);
    }
}

// --------------------------------------------------------------------------------

struct SearchNode
{
    SearchNode() {}

    SearchNode(Idx parent_, Idx relation_, bool inverse_)
        : parent(parent_), relation(relation_), inverse(inverse_) {}

    Idx parent;
    Idx relation;
    bool inverse;
};

// --------------------------------------------------------------------------------

bool WorldModel::calculateTransform(const UUID& source, const UUID& target, const Time& time, geo::Pose3D& tf) const
{
    Idx s, t;
    if (!findEntityIdx(source, s) || !findEntityIdx(target, t))
        return false;

    std::queue<Idx> Q;
    std::map<Idx, SearchNode> visited;

    Q.push(s);
    visited[s] = SearchNode(INVALID_IDX, INVALID_IDX, true);

    while(!Q.empty())
    {
        Idx n = Q.front();
        Q.pop();

        if (n == t)
        {
            // Calculate transformation
            tf = geo::Pose3D::identity();

            Idx u = n;

            while (u != s)
            {
                std::map<Idx, SearchNode>::const_iterator it = visited.find(u);
                const SearchNode& sn = it->second;

                const RelationConstPtr& r = relations_[sn.relation];

                geo::Pose3D tr;
                if (!r->calculateTransform(time, tr))
                {
                    std::cout << "WorldModel::calculateTransform: transform could not be calculated. THIS SHOULD NEVER HAPPEN!" << std::endl;
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
        for(std::map<Idx, Idx>::const_iterator it = transforms_to.begin(); it != transforms_to.end(); ++it)
        {
            Idx n2 = it->first;
            if (visited.find(n2) == visited.end())
            {
                visited[n2] = SearchNode(n, it->second, false);
                Q.push(n2);
            }
        }

        // Push all nodes this node points to
        const std::map<Idx, Idx>& transforms_from = entities_[n]->relationsFrom();
        for(std::map<Idx, Idx>::const_iterator it = transforms_from.begin(); it != transforms_from.end(); ++it)
        {
            Idx n2 = it->first;
            if (visited.find(n2) == visited.end())
            {
                visited[n2] = SearchNode(n, it->second, true);
                Q.push(n2);
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
        std::cout << "[ED] ERROR: Invalid relation addition: parent or child does not exit." << std::endl;
        return;
    }

    Idx r_idx = p->relationTo(child);
    if (r_idx == INVALID_IDX)
    {
        r_idx = addRelation(r);

        EntityPtr p_new(new Entity(*entities_[parent]));
        EntityPtr c_new(new Entity(*entities_[child]));

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
    for(std::size_t i = entity_revisions_.size(); i < std::max(parent, child) + 1; ++i)
        entity_revisions_.push_back(0);
    entity_revisions_[parent] = revision_;
    entity_revisions_[child] = revision_;
}

// --------------------------------------------------------------------------------

Idx WorldModel::addRelation(const RelationConstPtr& r)
{
    Idx r_idx = relations_.size();
    relations_.push_back(r);
    return r_idx;
}

// --------------------------------------------------------------------------------

void WorldModel::setEntity(const UUID& id, const EntityConstPtr& e)
{
    std::map<UUID, Idx>::const_iterator it_idx = entity_map_.find(id);
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
    std::map<UUID, Idx>::iterator it_idx = entity_map_.find(id);
    if (it_idx != entity_map_.end())
    {
        entities_[it_idx->second].reset();
        entity_revisions_[it_idx->second] = revision_;
        entity_shape_revisions_[it_idx->second] = 0;
        entity_empty_spots_.push(it_idx->second);
        entity_map_.erase(it_idx);
    }
}

// --------------------------------------------------------------------------------

EntityPtr WorldModel::getOrAddEntity(const UUID& id, std::map<UUID, EntityPtr>& new_entities)
{
    // Check if the id is already in the new_entities map. If so, return it
    std::map<UUID, EntityPtr>::const_iterator it_e = new_entities.find(id);
    if (it_e != new_entities.end())
        return it_e->second;

    EntityPtr e;

    Idx idx;
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

    for(std::size_t i = entity_revisions_.size(); i < idx + 1; ++i)
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

    std::map<UUID, Idx>::const_iterator it = entity_map_.find(id);
    if (it == entity_map_.end())
        return false;

    idx = it->second;
    id.idx = idx;
    return true;
}

// --------------------------------------------------------------------------------

Idx WorldModel::addNewEntity(const EntityConstPtr& e)
{
    Idx idx;
    if (entity_empty_spots_.empty())
    {
        idx = entities_.size();
        entity_map_[e->id()] = idx;
        entities_.push_back(e);
        entity_shape_revisions_.push_back(0);
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
        return 0;

    return property_info_db_->getPropertyKeyDBEntry(name);
}

}


