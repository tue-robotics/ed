#include "ed/io/transport/probe.h"

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <tue/serialization/conversions.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Probe::Probe()
{
}

// ----------------------------------------------------------------------------------------------------

Probe::~Probe()
{
}

// ----------------------------------------------------------------------------------------------------

void Probe::initialize()
{
    ros::NodeHandle nh;

    ros::AdvertiseServiceOptions opt_srv =
            ros::AdvertiseServiceOptions::create<tue_serialization::BinaryService>(
                "ed/probe/" + name(), boost::bind(&Probe::srvCallback, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_ = nh.advertiseService(opt_srv);

    std::cout << "Probe '" << name() << "' initialized." << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void Probe::process(const WorldModel& world, UpdateRequest& req)
{
    world_ = &world;
    update_req_ = &req;

    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool Probe::srvCallback(const tue_serialization::BinaryService::Request& ros_req,
                        tue_serialization::BinaryService::Response& ros_res)
{
    std::stringstream ss_req;
    tue::serialization::convert(ros_req.bin.data, ss_req);
    tue::serialization::InputArchive req(ss_req);

    std::stringstream ss_res;
    tue::serialization::OutputArchive res(ss_res);

    this->process(*world_, *update_req_, req, res);

    tue::serialization::convert(ss_res, ros_res.bin.data);

    return true;
}

}
