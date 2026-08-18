#include <rclcpp/rclcpp.hpp>

#include <ed_interfaces/srv/get_gui_command.hpp>
#include <ed_interfaces/srv/get_measurements.hpp>
#include <ed_interfaces/srv/raise_event.hpp>
#include <ed_interfaces/srv/set_label.hpp>
#include <ed_interfaces/srv/simple_query.hpp>

#include <tue/profiling/timer.h>

template <typename SrvT> void timeService(const rclcpp::Node::SharedPtr& node, const std::string& name, int N)
{
    auto client = node->create_client<SrvT>(name);
    client->wait_for_service();

    tue::Timer t;
    t.start();

    for (int i = 0; i < N; ++i)
    {
        auto request = std::make_shared<typename SrvT::Request>();
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS)
            std::cout << name << " : could not be called" << std::endl;
    }

    std::cout << name << ": " << t.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr const node = rclcpp::Node::make_shared("ed_test_service_speed");

    int const N = 1;

    timeService<ed_interfaces::srv::SimpleQuery>(node, "/ed/simple_query", N);
    timeService<ed_interfaces::srv::SetLabel>(node, "/ed/gui/set_label", N);
    timeService<ed_interfaces::srv::GetMeasurements>(node, "/ed/gui/get_measurements", N);
    timeService<ed_interfaces::srv::GetGUICommand>(node, "/ed/gui/get_gui_command", N);
    timeService<ed_interfaces::srv::RaiseEvent>(node, "/ed/gui/raise_event", N);

    rclcpp::shutdown();
    return 0;
}
