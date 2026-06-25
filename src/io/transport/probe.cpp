#include "ed/io/transport/probe.h"
#include "ed/types.h"
#include "tue_serialization_interfaces/srv/binary_service.hpp"

#include <iostream>
#include <memory>
#include <ostream>
#include <rclcpp/callback_group.hpp>
#include <sstream>
#include <tue/serialization/conversions.h>

#include <functional>
#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Probe::Probe() = default;

// ----------------------------------------------------------------------------------------------------

Probe::~Probe() = default;

// ----------------------------------------------------------------------------------------------------

void Probe::initialize()
{
    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // std::bind is required here: rclcpp deduces the service callback signature from the concrete
    // argument types, which a generic `auto&&` lambda does not provide.
    srv_ = node_->create_service<tue_serialization_interfaces::srv::BinaryService>(
        "ed/probe/" + name(),
        std::bind(
            &Probe::srvCallback, this, std::placeholders::_1, std::placeholders::_2), // NOLINT(modernize-avoid-bind)
        rclcpp::ServicesQoS(),
        cb_group_);

    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());

    std::cout << "Probe '" << name() << "' initialized." << '\n';
}

// ----------------------------------------------------------------------------------------------------

void Probe::process(const WorldModel& world, UpdateRequest& req)
{
    world_ = &world;
    update_req_ = &req;

    executor_.spin_some();
}

// ----------------------------------------------------------------------------------------------------

// NOLINTNEXTLINE(performance-unnecessary-value-param) - rclcpp service callback requires shared_ptr by value
void Probe::srvCallback(const std::shared_ptr<tue_serialization_interfaces::srv::BinaryService::Request> ros_req,
                        const std::shared_ptr<tue_serialization_interfaces::srv::BinaryService::Response>& ros_res)
{
    std::stringstream ss_req;
    tue::serialization::convert(ros_req->bin.data, ss_req);
    tue::serialization::InputArchive req(ss_req);

    std::stringstream ss_res;
    tue::serialization::OutputArchive res(ss_res);

    this->process(*world_, *update_req_, req, res);

    tue::serialization::convert(ss_res, ros_res->bin.data);
}

} // namespace ed
