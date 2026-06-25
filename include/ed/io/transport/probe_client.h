#ifndef ED_PROBE_CLIENT_H_
#define ED_PROBE_CLIENT_H_

#include <tue/serialization/archive.h>

#include <tue/config/configuration.h>

#include <rclcpp/rclcpp.hpp>
#include <tue_serialization_interfaces/srv/binary_service.hpp>

namespace ed
{

class ProbeClient
{

public:
    ProbeClient();

    virtual ~ProbeClient();

    void launchProbe(const std::string& probe_name, const std::string& lib);

    void configure(const tue::Configuration& config);

    bool process(tue::serialization::Archive& req, tue::serialization::Archive& res);

    [[nodiscard]]
    const std::string& probeName() const
    {
        return probe_name_;
    }

private:
    rclcpp::Node::SharedPtr node_;

    std::string probe_name_;

    rclcpp::Client<tue_serialization_interfaces::srv::BinaryService>::SharedPtr srv_probe_;
};

} // namespace ed

#endif
