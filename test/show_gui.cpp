#include <ed_interfaces/srv/raise_event.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tue_serialization_interfaces/msg/binary.hpp>

#include <vector>

rclcpp::Node::SharedPtr g_node;
rclcpp::Client<ed_interfaces::srv::RaiseEvent>::SharedPtr client;

std::string click_type;

void imageCallback(const tue_serialization_interfaces::msg::Binary::ConstSharedPtr msg)
{
    // Copy into a vector first: on Rolling a uint8[] field is rosidl::Buffer<uint8_t>,
    // which cv::InputArray cannot be constructed from. std::vector works on every distro.
    std::vector<unsigned char> const buf(msg->data.begin(), msg->data.end());
    cv::Mat const image = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    cv::imshow("map", image);
    cv::waitKey(3);
}

void mouseCallback(int event, int x, int y, int /*flags*/, void* /*ptr*/)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        auto srv_ev = std::make_shared<ed_interfaces::srv::RaiseEvent::Request>();
        srv_ev->name = "click";
        srv_ev->param_names.push_back("x");
        srv_ev->param_names.push_back("y");

        srv_ev->param_values.push_back(std::to_string(x));
        srv_ev->param_values.push_back(std::to_string(y));

        srv_ev->param_names.push_back("type");
        srv_ev->param_values.push_back(click_type);

        // Fire-and-forget: this callback runs inside the executor spin, so we cannot block on the result here.
        client->async_send_request(srv_ev);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    g_node = rclcpp::Node::make_shared("ed_gui");
    auto sub_image =
        g_node->create_subscription<tue_serialization_interfaces::msg::Binary>("/ed/gui/map_image", 1, imageCallback);
    client = g_node->create_client<ed_interfaces::srv::RaiseEvent>("/ed/gui/raise_event");

    click_type = "navigate";
    if (argc >= 2)
    {
        click_type = argv[1];
    }

    cv::namedWindow("map", 1);

    cv::setMouseCallback("map", mouseCallback);

    rclcpp::WallRate r(30);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(g_node);
        r.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
