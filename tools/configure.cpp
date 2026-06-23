#include <rclcpp/rclcpp.hpp>

#include <ed_interfaces/srv/configure.hpp>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include <tue/config/resolve_config.h>

#include <chrono>
#include <filesystem>

using namespace std::chrono_literals;

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: configure CONFIG_FILE.yaml/json" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    std::vector<std::string> myargv = rclcpp::init_and_remove_ros_arguments(argc, argv);
    if (myargv.size() != 2)
    {
        usage();
        return 1;
    }

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("ed_configure");
    rclcpp::Client<ed_interfaces::srv::Configure>::SharedPtr client =
            node->create_client<ed_interfaces::srv::Configure>("ed/configure");

    std::filesystem::path config_file(myargv[1]);
    if (!std::filesystem::exists(config_file))
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not configure ED: config file '" << config_file.string() << "' does not exist");
        return 1;
    }

    tue::config::ResolveConfig resolve_config;
    resolve_config.env = true;
    resolve_config.file = false;
    resolve_config.rospkg = false;
    tue::Configuration config;
    if (!tue::config::loadFromYAMLFile(config_file.string(), config, resolve_config))
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not configure ED: Error during parsing of the config file '" << config_file.string() << "' "<< std::endl << std::endl << config.error());
        return 1;
    }

    auto request = std::make_shared<ed_interfaces::srv::Configure::Request>();
    request->request = config.toYAMLString();

    // We do this as late as possible, so as much time as possible has passed doing other stuff
    // and we wait as less as possible.
    client->wait_for_service();

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not configure ED: Service call failed");
        return 1;
    }

    auto response = future.get();
    if (!response->error_msg.empty())
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Could not configure ED:\n\n" + response->error_msg);
        return 1;
    }

    return 0;
}
