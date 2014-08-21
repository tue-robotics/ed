#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <tue_serialization/Binary.h>
#include <ed/RaiseEvent.h>

ros::ServiceClient client;

std::string click_type;

void imageCallback(const tue_serialization::Binary::ConstPtr& msg)
{
    cv::Mat image = cv::imdecode(msg->data, CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow("map", image);
    cv::waitKey(3);
}

void mouseCallback(int event, int x, int y, int flags, void* ptr)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        ed::RaiseEvent srv_ev;
        srv_ev.request.name = "click";
        srv_ev.request.param_names.push_back("x");
        srv_ev.request.param_names.push_back("y");

        std::stringstream x_str;
        x_str << x;
        std::stringstream y_str;
        y_str << y;

        srv_ev.request.param_values.push_back(x_str.str());
        srv_ev.request.param_values.push_back(y_str.str());

        srv_ev.request.param_names.push_back("type");
        srv_ev.request.param_values.push_back(click_type);

        if (client.call(srv_ev))
        {
            std::cout << "Response from server: " << srv_ev.response.msg << std::endl;
        }
        else
        {
            std::cout << "Calling raise event failed" << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_gui");

    ros::NodeHandle nh;
    ros::Subscriber sub_image = nh.subscribe("/ed/gui/map_image", 1, imageCallback);
    client = nh.serviceClient<ed::RaiseEvent>("/ed/gui/raise_event");

    click_type = "navigate";
    if (argc >= 2)
    {
        click_type = argv[1];
    }


    cv::namedWindow("map", 1);

    cv::setMouseCallback("map", mouseCallback);


    ros::Rate r(30);
    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
