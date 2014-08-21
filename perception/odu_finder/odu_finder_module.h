#ifndef ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_
#define ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_

#include <ed/perception_modules/perception_module.h>

namespace odu_finder
{
class ODUFinder;
}

class ODUFinderModule : public ed::PerceptionModule
{

public:

    ODUFinderModule();

    virtual ~ODUFinderModule();

    void loadConfig(const std::string& config_path);

    ed::PerceptionResult process(const ed::Measurement& msr) const;

private:

    std::string config_path_;

//    odu_finder::ODUFinder* odu_finder_;

};

#endif
