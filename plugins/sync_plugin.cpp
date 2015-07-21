#include "sync_plugin.h"

#include <ros/node_handle.h>

#include "ed/Query.h"
#include "ed/io/json_reader.h"
#include "ed/update_request.h"

#include "ed/world_model.h"

#include <iostream>

#include "../examples/custom_properties/pose_info.h"
#include "../examples/custom_properties/counter_info.h"

#include "ed/convex_hull_calc.h"

#include "geolib/Shape.h"

// ----------------------------------------------------------------------------------------------------

SyncPlugin::SyncPlugin() : rev_number_(0)
{
}

// ----------------------------------------------------------------------------------------------------

SyncPlugin::~SyncPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void SyncPlugin::initialize(ed::InitData& init)
{
    std::string server_name;
    init.config.value("server", server_name);

    ros::NodeHandle nh;
    sync_client_ = nh.serviceClient<ed::Query>(server_name);

    ed::PropertyKey<geo::Pose3D> k_pose;
    ed::PropertyKey<int> k_counter;

    init.properties.registerProperty("pose", k_pose, new PoseInfo);
    init.properties.registerProperty("counter", k_counter, new CounterInfo);
}

// ----------------------------------------------------------------------------------------------------

void SyncPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    ed::Query query;
    query.request.since_revision = rev_number_;

    if (!sync_client_.call(query))
    {
        ROS_ERROR_STREAM("[ED SyncPlugin] Failed to call service '" << sync_client_.getService() << "'");
        return;
    }

    ed::io::JSONReader r(query.response.human_readable.c_str());

    if (!r.ok())
    {
        ROS_ERROR_STREAM("[ED SyncPlugin] Could not parse query response received from '" << sync_client_.getService() << "': " << query.response);
        return;
    }

//    std::cout << "Response size: " << query.response.human_readable.size() << std::endl;

    std::string error;

    if (r.readArray("entities"))
    {
        while(r.nextArrayItem())
        {
            std::string id;
            if (!r.readValue("id", id))
            {
                error += "Entities should have field 'id'.\n";
                continue;
            }

            std::string type;
            if (r.readValue("type", type))
                req.setType(id, type);

            double existence_prob;
            if (r.readValue("existence_prob", existence_prob))
                req.setExistenceProbability(id, existence_prob);

            double timestamp = 0;
            if (r.readGroup("timestamp"))
            {
                int sec, nsec;
                r.readValue("sec", sec);
                r.readValue("nsec", nsec);
                timestamp = sec + (double)nsec / 1e9;
                req.setLastUpdateTimestamp(id, timestamp);
                r.endGroup();
            }

            geo::Pose3D pose = geo::Pose3D::identity();
            if (r.readGroup("pose"))
            {
                if (r.readGroup("pos"))
                {
                    r.readValue("x", pose.t.x);
                    r.readValue("y", pose.t.y);
                    r.readValue("z", pose.t.z);
                    r.endGroup();
                }

                if (r.readGroup("rot"))
                {
                    r.readValue("xx", pose.R.xx);
                    r.readValue("xy", pose.R.xy);
                    r.readValue("xz", pose.R.xz);
                    r.readValue("yx", pose.R.yx);
                    r.readValue("yy", pose.R.yy);
                    r.readValue("yz", pose.R.yz);
                    r.readValue("zx", pose.R.zx);
                    r.readValue("zy", pose.R.zy);
                    r.readValue("zz", pose.R.zz);
                    r.endGroup();
                }

                req.setPose(id, pose);

                r.endGroup();
            }

            if (r.readGroup("convex_hull"))
            {
                ed::ConvexHull chull;
                if (r.readArray("points"))
                {
                    while(r.nextArrayItem())
                    {
                        geo::Vec2f p;
                        r.readValue("x", p.x);
                        r.readValue("y", p.y);
                        chull.points.push_back(p);
                    }
                    r.endArray();
                }

                r.readValue("z_min", chull.z_min);
                r.readValue("z_max", chull.z_max);

                ed::convex_hull::calculateEdgesAndNormals(chull);
                req.setConvexHullNew(id, chull, pose, timestamp);

                r.endGroup();
            }

            if (r.readGroup("mesh"))
            {
                geo::Mesh mesh;

                // Vertices
                if (r.readArray("vertices"))
                {
                    while(r.nextArrayItem())
                    {
                        geo::Vector3 p;
                        r.readValue("x", p.x);
                        r.readValue("y", p.y);
                        r.readValue("z", p.z);
                        mesh.addPoint(p);
                    }

                    r.endArray();
                }

                // Triangles
                if (r.readArray("triangles"))
                {
                    while(r.nextArrayItem())
                    {
                        int i1, i2, i3;
                        r.readValue("i1", i1);
                        r.readValue("i2", i2);
                        r.readValue("i3", i3);
                        mesh.addTriangle(i1, i2, i3);
                    }

                    r.endArray();
                }

                geo::ShapePtr shape(new geo::Shape);
                shape->setMesh(mesh);
                req.setShape(id, shape);

                r.endGroup();
            }

            if (r.readArray("properties"))
            {
                while(r.nextArrayItem())
                {
                    std::string prop_name;
                    if (!r.readValue("name", prop_name))
                        continue;

                    const ed::PropertyKeyDBEntry* entry = data.world.getPropertyInfo(prop_name);
                    if (!entry)
                    {
                        error += "For entity '" + id + "': unknown property '" + prop_name +"'.\n";
                        continue;
                    }

                    if (!entry->info->serializable())
                    {
                        error += "For entity '" + id + "': property '" + prop_name +"' is not serializable.\n";
                        continue;
                    }

                    ed::Variant value;
                    if (entry->info->deserialize(r, value))
                        req.setProperty(id, entry, value);
                    else
                        error += "For entity '" + id + "': deserialization of property '" + prop_name +"' failed.\n";
                }

                r.endArray();
            }
        }

        r.endArray();
    }

    if (!r.ok() || !error.empty())
    {
        ROS_ERROR_STREAM("[ED SyncPlugin] Invalid query response from '" << sync_client_.getService() << "': " << error);

        // Clear update request
        req = ed::UpdateRequest();
    }
    else
    {
        rev_number_ = query.response.new_revision;
    }
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(SyncPlugin)
