#include "ed/world_model.h"

#include "ed/update_request.h"
#include "ed/entity.h"
#include "ed/relation.h"

#include <tue/config/reader.h>

namespace ed
{

// --------------------------------------------------------------------------------

void WorldModel::update(const UpdateRequest& req)
{
    if (req.empty())
        return;

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

    // Update shapes
    for(std::map<UUID, geo::ShapeConstPtr>::const_iterator it = req.shapes.begin(); it != req.shapes.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setShape(it->second);
    }

    // Update types
    for(std::map<UUID, std::string>::const_iterator it = req.types.begin(); it != req.types.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setType(it->second);
    }

    // Update poses
    for(std::map<UUID, geo::Pose3D>::const_iterator it = req.poses.begin(); it != req.poses.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);
        e->setPose(it->second);
    }

    // Update additional info (data)
    for(std::map<UUID, tue::config::DataConstPointer>::const_iterator it = req.datas.begin(); it != req.datas.end(); ++it)
    {
        EntityPtr e = getOrAddEntity(it->first, new_entities);

            EntityPtr e_updated(new Entity(*e));

            tue::config::DataPointer params;
            params.add(e->data());
            params.add(it->second);

            tue::config::Reader r(params);
            std::string type;
            if (r.value("type", type, tue::config::OPTIONAL))
                e_updated->setType(type);

            e_updated->setData(params);

            setEntity(it->first, e_updated);

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

            while (true)
            {
                std::map<Idx, SearchNode>::const_iterator it = visited.find(u);
                const SearchNode& sn = it->second;

                if (u == s)
                    break;

                const RelationConstPtr& r = relations_[sn.relation];

                geo::Pose3D tr;
                if (!r->calculateTransform(time, tr))
                {
                    std::cout << "WorldModel::calculateTransform: transform could not be calculated. THIS SHOULD NEVER HAPPEN!" << std::endl;
                    return false;
                }

                if (sn.inverse)
                    tf = tf * tr.inverse();
                else
                    tf = tf * tr;

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

        // Push all nodes that point to this node
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

void WorldModel::setRelation(Idx parent, Idx child, const RelationPtr& r)
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
}

// --------------------------------------------------------------------------------

Idx WorldModel::addRelation(const RelationPtr& r)
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
        entity_map_.erase(it_idx);
        entity_empty_spots_.push(it_idx->second);
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

    std::map<UUID, Idx>::const_iterator it_idx = entity_map_.find(id);
    if (it_idx == entity_map_.end())
    {
        // Does not yet exist
        e = boost::make_shared<Entity>(id);
        addNewEntity(e);
    }
    else
    {
        // Already exists
        Idx idx = it_idx->second;

        // Create a copy of the existing entity
        e = boost::make_shared<Entity>(*entities_[idx]);

        // Set the copy
        entities_[idx] = e;

        new_entities[id] = e;
    }

    return e;
}

// --------------------------------------------------------------------------------

void WorldModel::addNewEntity(const EntityConstPtr& e)
{
    if (entity_empty_spots_.empty())
    {
        Idx idx = entities_.size();
        entity_map_[e->id()] = idx;
        entities_.push_back(e);
    }
    else
    {
        Idx idx = entity_empty_spots_.front();
        entity_empty_spots_.pop();
        entity_map_[e->id()] = idx;
        entities_[idx] = e;
    }
}

// --------------------------------------------------------------------------------


}


