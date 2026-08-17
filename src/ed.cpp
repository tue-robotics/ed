#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <pthread.h>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>

#include "ed/property.h"
#include "ed/server.h"

#include <ed/world_model.h>

// Query
#include <ed/entity.h>
#include <ed/helpers/msg_conversions.h>
#include <ed/serialization/serialization.h>
#include <ed_interfaces/srv/simple_query.hpp>
#include <geolib/datatypes.h>
#include <geolib/ros/msg_conversions.h>
#include <rclcpp/utilities.hpp>
#include <sstream>
#include <string>
#include <tue/config/configuration.h>
#include <tue/config/yaml_emitter.h>

#include "ed/io/json_writer.h"
#include <ed_interfaces/srv/query.hpp>

// Update
#include "ed/io/json_reader.h"
#include "ed/types.h"
#include "ed/update_request.h"
#include <ed_interfaces/srv/update_srv.hpp>

// Reset
#include <ed_interfaces/srv/reset.hpp>

// Configure
#include <ed_interfaces/srv/configure.hpp>

// Loop
#include <ed/event_clock.h>

// Plugin loading
#include <tue/config/loaders/yaml.h>

#include "ed/error_context.h"
#include "ed/variant.h"
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <set>
#include <thread>
#include <unistd.h>

#include <vector>

namespace
{

// Initialized during static initialization, which runs on the main thread.
const std::thread::id MAIN_THREAD_ID = std::this_thread::get_id();

// ----------------------------------------------------------------------------------------------------

void srvReset(ed::Server& server,
              const std::shared_ptr<ed_interfaces::srv::Reset::Request>& req,
              const std::shared_ptr<ed_interfaces::srv::Reset::Response>& /*res*/)
{
    server.reset(req->keep_all_shapes);
}

// ----------------------------------------------------------------------------------------------------

void srvUpdate(ed::Server& server,
               const std::shared_ptr<ed_interfaces::srv::UpdateSrv::Request>& req,
               const std::shared_ptr<ed_interfaces::srv::UpdateSrv::Response>& res)
{
    ed::io::JSONReader r(req->request.c_str());

    if (!r.ok())
    {
        res->response = r.error();
        return;
    }

    ed::UpdateRequest update_req;

    if (r.readArray("entities"))
    {
        while (r.nextArrayItem())
        {
            std::string id;
            if (!r.readValue("id", id))
            {
                res->response += "Entities should have field 'id'.\n";
                continue;
            }

            std::string action;
            if (r.readValue("action", action))
            {
                if (action == "remove")
                    update_req.removeEntity(id);
                else
                    res->response += "Unknown action '" + action + "'.\n";
            }

            std::string type;
            if (r.readValue("type", type))
            {
                update_req.setType(id, type);
            }

            if (r.readGroup("pose"))
            {
                double x = NAN;
                double y = NAN;
                double z = NAN;
                std::string remove;
                if (r.readValue("x", x) && r.readValue("y", y) && r.readValue("z", z))
                {
                    geo::Pose3D pose = geo::Pose3D::identity();
                    pose.t = geo::Vector3(x, y, z);

                    double roll = NAN;
                    double pitch = NAN;
                    double yaw = NAN;
                    if (r.readValue("X", roll) && r.readValue("Y", pitch) && r.readValue("Z", yaw))
                        pose.setRPY(roll, pitch, yaw);

                    update_req.setPose(id, pose);
                }
                else if (r.readValue("remove", remove) && remove == "true")
                {
                    update_req.removePose(id);
                }
                else
                {
                    res->response += "For entity '" + id + "': invalid pose (position).\n";
                }

                r.endGroup();
            }

            if (r.readArray("flags"))
            {
                while (r.nextArrayItem())
                {
                    std::string flag;
                    if (r.readValue("add", flag))
                        update_req.setFlag(id, flag);
                    else if (r.readValue("remove", flag))
                        update_req.removeFlag(id, flag);
                    else
                        res->response += "For entity '" + id + "': flag list should only contain 'add' or 'remove'.\n";
                }
            }

            // Add data of entity, which is used for extra properties
            // ToDo: should data be used in this way? Or should other variables be introduced for this purpose
            std::string data_str;
            if (r.readValue("data", data_str))
            {
                tue::Configuration data_config;
                if (tue::config::loadFromYAMLString(data_str, data_config))
                {
                    update_req.addData(id, data_config.data());
                }
            }

            if (r.readArray("properties"))
            {
                while (r.nextArrayItem())
                {
                    std::string prop_name;
                    if (!r.readValue("name", prop_name))
                        continue;

                    // ToDo: is this thread safe?
                    const ed::PropertyKeyDBEntry* entry = server.getPropertyKeyDBEntry(prop_name);
                    if (!entry)
                    {
                        res->response.append("For entity '")
                            .append(id)
                            .append("': unknown property '")
                            .append(prop_name)
                            .append("'.\n");
                        continue;
                    }

                    if (!entry->info->serializable())
                    {
                        res->response.append("For entity '")
                            .append(id)
                            .append("': property '")
                            .append(prop_name)
                            .append("' is not serializable.\n");
                        continue;
                    }

                    ed::Variant value;
                    if (entry->info->deserialize(r, value))
                        update_req.setProperty(id, entry, value);
                    else
                        res->response.append("For entity '")
                            .append(id)
                            .append("': deserialization of property '")
                            .append(prop_name)
                            .append("' failed.\n");
                }

                r.endArray();
            }
        }

        r.endArray();
    }

    if (r.ok())
    {
        if (!update_req.empty())
        {
            server.update(update_req);
        }
    }
    else
    {
        res->response += r.error();
    }
}

// ----------------------------------------------------------------------------------------------------

void srvQuery(ed::Server& server,
              const std::shared_ptr<ed_interfaces::srv::Query::Request>& req,
              const std::shared_ptr<ed_interfaces::srv::Query::Response>& res)
{
    // Set of queried ids
    std::set<std::string> ids(req->ids.begin(), req->ids.end());

    // convert property names to indexes
    std::vector<ed::Idx> property_idxs;
    for (const auto& propertie : req->properties)
    {
        // ToDo: is this thread safe?
        const ed::PropertyKeyDBEntry* entry = server.getPropertyKeyDBEntry(propertie);
        if (entry)
            property_idxs.push_back(entry->idx);
    }

    // Make a copy of the WM, to keep it thead safe
    ed::WorldModel const wm = *server.worldModel();
    const auto& entity_revs = wm.entityRevisions();
    const std::vector<ed::EntityConstPtr>& entities = wm.entities();

    std::vector<std::string> removed_entities;

    std::stringstream out;
    ed::io::JSONWriter w(out);

    w.writeArray("entities");

    for (ed::Idx i = 0; i < entity_revs.size(); ++i)
    {
        if (req->since_revision >= entity_revs[i])
            continue;

        const ed::EntityConstPtr& e = entities[i];
        if (!e)
            continue;

        if (!ids.empty() && ids.find(e->id().str()) == ids.end())
            continue;

        if (e)
        {
            w.addArrayItem();
            w.writeValue("id", e->id().str());
            w.writeValue("idx", static_cast<int>(i));

            // Write type
            w.writeValue("type", e->type());

            w.writeValue("existence_prob", e->existenceProbability());

            w.writeGroup("timestamp");
            {
                ed::serializeTimestamp(e->lastUpdateTimestamp(), w);
                w.endGroup();
            }

            // Write convex hull
            if (!e->convexHull().points.empty() && wm.entityVisualRevisions()[i] > req->since_revision)
            {
                w.writeGroup("convex_hull");
                ed::serialize(e->convexHull(), w);
                w.endGroup();
            }

            // Pose
            if (e->hasPose())
            {
                w.writeGroup("pose");
                ed::serialize(e->pose(), w);
                w.endGroup();
            }

            // Mesh
            if (e->visual() && wm.entityVisualRevisions()[i] > req->since_revision)
            {
                w.writeGroup("mesh");
                ed::serialize(*e->visual(), w);
                w.endGroup();
            }

            // Data
            if (!e->data().empty())
            {
                std::stringstream out;
                tue::config::YAMLEmitter::emit(e->data(), out);

                std::string data_str = out.str();

                std::replace(data_str.begin(), data_str.end(), '"', '|');
                std::replace(data_str.begin(), data_str.end(), '\n', '^');

                w.writeValue("data", data_str);
            }

            w.writeArray("properties");

            const std::map<ed::Idx, ed::Property>& properties = e->properties();

            if (req->properties.empty())
            {
                for (const auto& propertie : properties)
                {
                    const ed::Property& prop = propertie.second;
                    if (req->since_revision < prop.revision && prop.entry->info->serializable())
                    {
                        w.addArrayItem();
                        w.writeValue("name", prop.entry->name);
                        prop.entry->info->serialize(prop.value, w);
                        w.endArrayItem();
                    }
                }
            }
            else
            {
                for (ed::Idx const property_idx : property_idxs)
                {
                    auto const it_prop = properties.find(property_idx);
                    if (it_prop != properties.end())
                    {
                        const ed::Property& prop = it_prop->second;
                        if (req->since_revision < prop.revision && prop.entry->info->serializable())
                        {
                            w.addArrayItem();
                            w.writeValue("name", prop.entry->name);
                            prop.entry->info->serialize(prop.value, w);
                            w.endArrayItem();
                        }
                    }
                }
            }

            w.endArray();

            w.endArrayItem();
        }
        else
        {
            // Was removed
            removed_entities.push_back(e->id().str());
        }
    }

    w.endArray();

    if (!removed_entities.empty())
        w.writeValue("removed_entities", removed_entities.data(), removed_entities.size());

    w.finish();

    res->human_readable = out.str();
    res->new_revision = wm.revision();
}

// ----------------------------------------------------------------------------------------------------

void srvSimpleQuery(ed::Server& server,
                    const std::shared_ptr<ed_interfaces::srv::SimpleQuery::Request>& req,
                    const std::shared_ptr<ed_interfaces::srv::SimpleQuery::Response>& res)
{
    double const radius = req->radius;
    geo::Vector3 center_point;
    geo::convert(req->center_point, center_point);

    // Make a copy of the WM, to keep it thead safe
    ed::WorldModel const wm = *server.worldModel();
    for (const auto& e : wm)
    {
        if (!req->id.empty() && e->id() != ed::UUID(req->id))
            continue;

        if (!e->hasPose())
            continue;

        if (!req->type.empty())
        {
            if (req->type == "unknown")
            {
                if (!e->type().empty())
                    continue;
            }
            else
            {
                if (!e->hasType(req->type))
                    continue;
            }
        }

        if (radius < std::numeric_limits<double>::infinity())
        {
            bool geom_ok = false;

            if (req->ignore_z)
                center_point.z = e->pose().t.z; // Ignoring z in global frame, not in entity frame, as it can be rotated

            geo::ShapeConstPtr const visual = e->visual();
            if (visual)
            {
                geo::Vector3 const center_point_e =
                    e->pose().getBasis().transpose() * (center_point - e->pose().getOrigin());
                if (radius > 0)
                    geom_ok = visual->intersect(center_point_e, radius);
                else
                    geom_ok = visual->contains(center_point_e);
            }
            else
            {
                geom_ok = radius > 0 && radius * radius > (e->pose().t - center_point).length2();
            }

            if (!geom_ok)
                continue;
        }

        res->entities.emplace_back();
        convert(*e, res->entities.back());
    }
}

// ----------------------------------------------------------------------------------------------------

void srvConfigure(ed::Server& server,
                  const std::shared_ptr<ed_interfaces::srv::Configure::Request>& req,
                  const std::shared_ptr<ed_interfaces::srv::Configure::Response>& res)
{
    tue::Configuration config;
    if (!tue::config::loadFromYAMLString(req->request, config))
    {
        res->error_msg = config.error();
        return;
    }

    // Configure ED
    server.configure(config);

    if (config.hasError())
    {
        res->error_msg = config.error();
        return;
    }
}

// ----------------------------------------------------------------------------------------------------

void signalHandler(int sig)
{
    // Make sure to remove all signal handlers
    signal(SIGSEGV, SIG_DFL);
    signal(SIGABRT, SIG_DFL);

    std::cerr << "\033[38;5;1m";
    std::cerr << "[ED] ED Crashed!" << '\n' << '\n';

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print signal

    std::cerr << "    Signal: ";
    if (sig == SIGSEGV)
        std::cerr << "segmentation fault";
    else if (sig == SIGABRT)
        std::cerr << "abort";
    else
        std::cerr << "unknown";
    std::cerr << '\n' << '\n';

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print thread name

    std::cerr << "    Thread: ";

    char name[1000];
    size_t const name_size = 1000;
    if (pthread_getname_np(pthread_self(), name, name_size) == 0)
    {
        if (std::string(name) == "ed_main")
        {
            if (std::this_thread::get_id() == MAIN_THREAD_ID)
                std::cerr << "main";
            else
                std::cerr << "name unknown (id = " << std::this_thread::get_id() << ")";
        }
        else
            std::cerr << name;
    }
    else
        std::cerr << "name unknown (id = " << std::this_thread::get_id() << ")";

    std::cerr << '\n' << '\n';

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print error context

    std::cerr << "    Error context: ";

    ed::ErrorContextData* edata = ed::ErrorContext::data();
    if (edata && !edata->stack.empty())
    {
        std::cerr << '\n' << '\n';

        for (unsigned int i = edata->stack.size(); i > 0; --i)
        {
            const char* message = edata->stack[i - 1].first;
            const char* value = edata->stack[i - 1].second;

            if (message)
            {
                std::cerr << "        " << message;
                if (value)
                    std::cerr << " " << value;
                std::cerr << '\n';
            }
        }
    }
    else
        std::cerr << "unknown";

    std::cerr << '\n' << '\n';

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Print backtrace

    std::cerr << "--------------------------------------------------" << '\n';
    std::cerr << "Backtrace: " << '\n' << '\n';

    void* array[20];

    // get void*'s for all entries on the stack
    int const size = backtrace(array, 20);

    // print out all the frames to stderr
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    std::cerr << "\033[0m" << '\n';
    exit(1);
}

} // namespace

// ----------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr const node = rclcpp::Node::make_shared("ed");

    // Set the name of the main thread
    pthread_setname_np(pthread_self(), "ed_main");

    // register signal SIGINT and signal handler
    signal(SIGSEGV, signalHandler);
    signal(SIGABRT, signalHandler);

    ed::ErrorContext const errc("Start ED server", "init");

    // Create the ED server
    ed::Server server(node);

    // - - - - - - - - - - - - - - - configure - - - - - - - - - - - - - - -

    ed::ErrorContext::change("Start ED server", "configure");

    tue::Configuration config;

    // Check if a config file was provided. If so, load it. If not, load the default AMIGO config.
    if (argc >= 2)
    {
        std::string const yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);

        // Configure ED
        server.configure(config);

        if (config.hasError())
        {
            RCLCPP_ERROR_STREAM(node->get_logger(),
                                '\n' << "Error during configuration:" << '\n'
                                     << '\n'
                                     << config.error());
            return 1;
        }
    }

    // - - - - - - - - - - - - service initialization - - - - - - - - - - - -

    ed::ErrorContext::change("Start ED server", "service init");

    rclcpp::CallbackGroup::SharedPtr const cb_group =
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto srv_simple_query = node->create_service<ed_interfaces::srv::SimpleQuery>(
        "~/simple_query",
        [&server](const std::shared_ptr<ed_interfaces::srv::SimpleQuery::Request>& req,
                  const std::shared_ptr<ed_interfaces::srv::SimpleQuery::Response>& res)
        { srvSimpleQuery(server, req, res); },
        rclcpp::ServicesQoS(),
        cb_group);
    auto srv_reset = node->create_service<ed_interfaces::srv::Reset>(
        "~/reset",
        [&server](const std::shared_ptr<ed_interfaces::srv::Reset::Request>& req,
                  const std::shared_ptr<ed_interfaces::srv::Reset::Response>& res) { srvReset(server, req, res); },
        rclcpp::ServicesQoS(),
        cb_group);
    auto srv_query = node->create_service<ed_interfaces::srv::Query>(
        "~/query",
        [&server](const std::shared_ptr<ed_interfaces::srv::Query::Request>& req,
                  const std::shared_ptr<ed_interfaces::srv::Query::Response>& res) { srvQuery(server, req, res); },
        rclcpp::ServicesQoS(),
        cb_group);
    auto srv_update = node->create_service<ed_interfaces::srv::UpdateSrv>(
        "~/update",
        [&server](const std::shared_ptr<ed_interfaces::srv::UpdateSrv::Request>& req,
                  const std::shared_ptr<ed_interfaces::srv::UpdateSrv::Response>& res) { srvUpdate(server, req, res); },
        rclcpp::ServicesQoS(),
        cb_group);
    auto srv_configure = node->create_service<ed_interfaces::srv::Configure>(
        "~/configure",
        [&server](const std::shared_ptr<ed_interfaces::srv::Configure::Request>& req,
                  const std::shared_ptr<ed_interfaces::srv::Configure::Response>& res)
        { srvConfigure(server, req, res); },
        rclcpp::ServicesQoS(),
        cb_group);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(cb_group, node->get_node_base_interface());

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ed::ErrorContext::change("Start ED server", "init");

    // Init ED
    server.initialize();

    ed::EventClock trigger_config(10);
    ed::EventClock trigger_ed(10);
    ed::EventClock trigger_stats(2);
    ed::EventClock trigger_plugins(1000);
    ed::EventClock trigger_cb(100);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ed::ErrorContext::change("ED server", "main loop");

    rclcpp::WallRate r(1000);
    while (rclcpp::ok())
    {

        if (trigger_cb.triggers())
            executor.spin_some();

        // Check if configuration has changed. If so, call reconfigure
        if (trigger_config.triggers() && config.sync())
            server.configure(config, true);

        if (trigger_ed.triggers())
            server.update();

        if (trigger_plugins.triggers())
            server.stepPlugins();

        if (trigger_stats.triggers())
            server.publishStatistics();

        r.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
