#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>

#include <ed_msgs/Configure.h>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>

#include <tue/filesystem/path.h>

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: configure CONFIG_FILE.yaml/json" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    std::vector<std::string> myargv;
    ros::removeROSArgs(argc, argv, myargv);
    if (myargv.size() != 2)
    {
        usage();
        return 1;
    }

    ros::init(argc, argv, "ed_configure");

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<ed_msgs::Configure>("ed/configure");

    tue::filesystem::Path config_file(myargv[1]);
    if (!config_file.exists())
    {
        ROS_ERROR_STREAM("Could not configure ED: config file '" << config_file.string() << "' does not exist");
        return 1;
    }

    tue::Configuration config;
    if (!tue::config::loadFromYAMLFile(config_file.string(), config))
    {
        ROS_ERROR_STREAM("Could not configure ED: Error during parsing of the config file '" << config_file.string() << "' "<< std::endl << std::endl << config.error());
        return 1;
    }

    ed_msgs::Configure srv;
    srv.request.request = config.toYAMLString();

    // We do this as late as possible, so as much time as possible has passed doing other stuff
    // and we wait as less as possible.
    client.waitForExistence();

    if (!client.call(srv))
    {
        ROS_ERROR_STREAM("Could not configure ED: Service call failed");
        return 1;
    }

    if (!srv.response.error_msg.empty())
    {
        ROS_ERROR_STREAM("Could not configure ED:\n\n" + srv.response.error_msg);
        return 1;
    }

    return 0;
}
