#ifndef ED_PROBE_CLIENT_H_
#define ED_PROBE_CLIENT_H_

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

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

    bool process(std::stringstream& req,
                 tue::serialization::InputArchive& res);

    const std::string& probeName() const { return probe_name_; }

private:

    std::string probe_name_;

    ros::ServiceClient srv_probe_;

};

}

#endif
