#ifndef ED_PROBE_CLIENT_H_
#define ED_PROBE_CLIENT_H_

#include <tue/serialization/archive.h>

#include <tue/config/configuration.h>

#include <ros/service_client.h>

namespace ed
{

class ProbeClient
{

public:

    ProbeClient();

    virtual ~ProbeClient();

    void launchProbe(const std::string& probe_name, const std::string& lib);

    void configure(tue::Configuration config);

    bool process(tue::serialization::Archive& req, tue::serialization::Archive& res);

    const std::string& probeName() const { return probe_name_; }

private:

    ros::NodeHandle* nh_;

    std::string probe_name_;

    ros::ServiceClient srv_probe_;

};

}

#endif
