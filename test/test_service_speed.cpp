#include <ed_msgs/SetLabel.h>
#include <ed_msgs/SimpleQuery.h>
#include <ed_msgs/SetClick.h>
#include <ed_msgs/GetGUICommand.h>
#include <ed_msgs/GetMeasurements.h>
#include <ed_msgs/RaiseEvent.h>

#include <ros/ros.h>

#include <tue/profiling/timer.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ed_test_service_speed");

    ros::NodeHandle nh;

    int N = 1;

    {
        ros::ServiceClient client = nh.serviceClient<ed_msgs::SimpleQuery>("/ed/simple_query");
        client.waitForExistence();
        ed_msgs::SimpleQuery srv;

        tue::Timer t;
        t.start();

        for(int i = 0; i < N; ++i)
        {

            if (!client.call(srv))
            {
                std::cout << client.getService() << " : could not be called" << std::endl;
            }
        }

        std::cout << client.getService() << ": " << t.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
    }

    {
        ros::ServiceClient client = nh.serviceClient<ed_msgs::SetLabel>("/ed/gui/set_label");
        client.waitForExistence();
        ed_msgs::SetLabel srv;

        tue::Timer t;
        t.start();

        for(int i = 0; i < N; ++i)
        {

            if (!client.call(srv))
            {
                std::cout << client.getService() << " : could not be called" << std::endl;
            }
        }

        std::cout << client.getService() << ": " << t.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
    }

    {
        ros::ServiceClient client = nh.serviceClient<ed_msgs::GetMeasurements>("/ed/gui/get_measurements");
        client.waitForExistence();
        ed_msgs::GetMeasurements srv;

        tue::Timer t;
        t.start();

        for(int i = 0; i < N; ++i)
        {

            if (!client.call(srv))
            {
                std::cout << client.getService() << " : could not be called" << std::endl;
            }
        }

        std::cout << client.getService() << ": " << t.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
    }

    {
        ros::ServiceClient client = nh.serviceClient<ed_msgs::GetGUICommand>("/ed/gui/get_gui_command");
        client.waitForExistence();
        ed_msgs::GetGUICommand srv;

        tue::Timer t;
        t.start();

        for(int i = 0; i < N; ++i)
        {

            if (!client.call(srv))
            {
                std::cout << client.getService() << " : could not be called" << std::endl;
            }
        }

        std::cout << client.getService() << ": " << t.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
    }

    {
        ros::ServiceClient client = nh.serviceClient<ed_msgs::RaiseEvent>("/ed/gui/raise_event");
        client.waitForExistence();
        ed_msgs::RaiseEvent srv;

        tue::Timer t;
        t.start();

        for(int i = 0; i < N; ++i)
        {

            if (!client.call(srv))
            {
                std::cout << client.getService() << " : could not be called" << std::endl;
            }
        }

        std::cout << client.getService() << ": " << t.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
    }

    return 0;
}
