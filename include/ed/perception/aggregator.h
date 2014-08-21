#ifndef ED_PERCEPTION_AGGREGATOR_H_
#define ED_PERCEPTION_AGGREGATOR_H_

#include <ed/perception_modules/perception_module.h>

namespace ed
{

class PerceptionAggregator : public ed::PerceptionModule
{

public:

    PerceptionAggregator();

    virtual ~PerceptionAggregator();

    ed::PerceptionResult process(const ed::Measurement& msr) const;

    inline void addPerceptionModule(const ed::PerceptionModuleConstPtr& module)
    {
        perception_modules_.push_back(module);
    }

    inline void clear() { perception_modules_.clear(); }

protected:

    std::vector<ed::PerceptionModuleConstPtr> perception_modules_;

};

}

#endif
