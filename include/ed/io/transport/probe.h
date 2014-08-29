#ifndef ED_PROBE_H_
#define ED_PROBE_H_

#include "ed/plugin.h"

#include <tue/config/configuration.h>
#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <tue_serialization/BinaryService.h>

#include <ros/callback_queue.h>
#include <ros/service_server.h>

namespace ed
{

class Probe : public Plugin
{

public:

    Probe();

    virtual ~Probe();


    // Plugin interface

    void initialize();

    void process(const WorldModel& world, UpdateRequest& req);


    // Probe interface

    virtual void configure(tue::Configuration config) {}

    virtual void process(const WorldModel& world,
                         UpdateRequest& update,
                         tue::serialization::InputArchive& req,
                         tue::serialization::OutputArchive& res) {}

private:

    const ed::WorldModel* world_;
    ed::UpdateRequest* update_req_;

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_;

    bool srvCallback(const tue_serialization::BinaryService::Request& ros_req,
                     tue_serialization::BinaryService::Response& ros_res);

};

}

#endif
