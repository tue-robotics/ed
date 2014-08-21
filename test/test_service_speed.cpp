#include <ed/SetLabel.h>
#include <ed/SimpleQuery.h>
#include <ed/SetClick.h>
#include <ed/GetGUICommand.h>
#include <ed/GetMeasurements.h>
#include <ed/RaiseEvent.h>

#include <ros/ros.h>

#include <tue/profiling/timer.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ed_test_service_speed");

    ros::NodeHandle nh;

    int N = 1;

    {
        ros::ServiceClient client = nh.serviceClient<ed::SimpleQuery>("/ed/simple_query");
        client.waitForExistence();
        ed::SimpleQuery srv;

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
        ros::ServiceClient client = nh.serviceClient<ed::SetLabel>("/ed/gui/set_label");
        client.waitForExistence();
        ed::SetLabel srv;

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
        ros::ServiceClient client = nh.serviceClient<ed::GetMeasurements>("/ed/gui/get_measurements");
        client.waitForExistence();
        ed::GetMeasurements srv;

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
        ros::ServiceClient client = nh.serviceClient<ed::GetGUICommand>("/ed/gui/get_gui_command");
        client.waitForExistence();
        ed::GetGUICommand srv;

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
        ros::ServiceClient client = nh.serviceClient<ed::RaiseEvent>("/ed/gui/raise_event");
        client.waitForExistence();
        ed::RaiseEvent srv;

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
