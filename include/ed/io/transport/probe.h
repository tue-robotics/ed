#ifndef ED_PROBE_H_
#define ED_PROBE_H_

#include "ed/plugin.h"

#include <tue/config/configuration.h>
#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <tue_serialization_interfaces/srv/binary_service.hpp>

#include <rclcpp/rclcpp.hpp>

namespace ed
{

class Probe : public Plugin
{

public:
    Probe();

    ~Probe() override;

    // Plugin interface

    void initialize() override;

    void process(const WorldModel& world, UpdateRequest& req) override;

    // Probe interface

    void configure(tue::Configuration /*config*/) override {}

    using Plugin::process;

    virtual void process(const WorldModel& /*world*/,
                         UpdateRequest& /*update*/,
                         tue::serialization::InputArchive& /*req*/,
                         tue::serialization::OutputArchive& /*res*/)
    {
    }

private:
    const ed::WorldModel* world_{};
    ed::UpdateRequest* update_req_{};

    rclcpp::CallbackGroup::SharedPtr cb_group_;

    rclcpp::executors::SingleThreadedExecutor executor_;

    rclcpp::Service<tue_serialization_interfaces::srv::BinaryService>::SharedPtr srv_;

    // NOLINTNEXTLINE(performance-unnecessary-value-param) - rclcpp service callback requires shared_ptr by value
    void srvCallback(const std::shared_ptr<tue_serialization_interfaces::srv::BinaryService::Request>& ros_req,
                     const std::shared_ptr<tue_serialization_interfaces::srv::BinaryService::Response>& ros_res);
};

} // namespace ed

#endif
