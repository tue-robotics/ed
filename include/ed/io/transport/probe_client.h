#ifndef ED_PROBE_CLIENT_H_
#define ED_PROBE_CLIENT_H_

#include <tue/serialization/input_archive.h>
#include <tue/serialization/output_archive.h>

#include <tue/config/configuration.h>

namespace ed
{

class ProbeClient
{

public:

    ProbeClient();

    virtual ~ProbeClient();

    void launchProbe(const std::string& probe_name, const std::string& lib);

    void configure(tue::Configuration config);

    void process(const tue::serialization::OutputArchive& req,
                 tue::serialization::InputArchive& res);

};

}

#endif
