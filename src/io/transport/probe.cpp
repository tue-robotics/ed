#include "ed/io/transport/probe.h"

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <tue/serialization/conversions.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Probe::Probe()
{
    std::cout << "Probe::Probe" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

Probe::~Probe()
{
}

// ----------------------------------------------------------------------------------------------------

void Probe::initialize()
{
    std::cout << "Probe::initialize" << std::endl;

    ros::NodeHandle nh;

    ros::AdvertiseServiceOptions opt_srv =
            ros::AdvertiseServiceOptions::create<tue_serialization::BinaryService>(
                "/ed/probe/" + name(), boost::bind(&Probe::srvCallback, this, _1, _2),
                ros::VoidPtr(), &cb_queue_);

    srv_ = nh.advertiseService(opt_srv);
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
    std::cout << "Probe::srvCallback" << std::endl;

    std::stringstream ss_req;    
    tue::serialization::convert(ros_req.bin.data, ss_req);
    tue::serialization::InputArchive req(ss_req);

    std::cout << "FOOO: " << ros_req.bin.data.size() << " --- " << ss_req.str().size() << std::endl;

    std::stringstream ss_res;
    tue::serialization::OutputArchive res(ss_res, 0);

    this->process(*world_, *update_req_, req, res);

    tue::serialization::convert(ss_res, ros_res.bin.data);

    std::cout << "Probe::srvCallback - done" << std::endl;

    return true;
}

}
