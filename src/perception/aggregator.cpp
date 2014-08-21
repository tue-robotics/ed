#include "ed/perception/aggregator.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

void updatePerceptionResult(ed::PerceptionResult& res, const ed::PerceptionResult& update)
{
    std::map<std::string, Percept> orig_percepts = res.percepts();
    const std::map<std::string, Percept>& new_percepts = update.percepts();

    for(std::map<std::string, Percept>::const_iterator it = new_percepts.begin(); it != new_percepts.end(); ++it)
    {
        const Percept& p = it->second;

        std::map<std::string, Percept>::const_iterator it2 = orig_percepts.find(it->first);
        if (it2 == new_percepts.end())
        {
            res.addInfo(it->first, p.score, p.pose);
        }
        else
        {
            double score = it2->second.score;
            if (p.score > 0)
                score += p.score;
            else
                score = 0;

            res.addInfo(it->first, score, p.pose);
        }
    }

    // Add all visualizations
    const std::map<std::string, cv::Mat>& vis_images = update.visualizationImages();
    for(std::map<std::string, cv::Mat>::const_iterator it = vis_images.begin(); it != vis_images.end(); ++it)
    {
        res.setVisualization(it->first, it->second);
    }
}

// ----------------------------------------------------------------------------------------------------

PerceptionAggregator::PerceptionAggregator() :
    PerceptionModule("aggregator")
{
}

// ----------------------------------------------------------------------------------------------------

PerceptionAggregator::~PerceptionAggregator()
{
}

// ----------------------------------------------------------------------------------------------------

ed::PerceptionResult PerceptionAggregator::process(const ed::Measurement& msr) const
{
    ed::PerceptionResult res;

    for(std::vector<ed::PerceptionModuleConstPtr>::const_iterator it = perception_modules_.begin(); it != perception_modules_.end(); ++it)
    {
        ed::PerceptionResult sub_res = (*it)->process(msr);
        updatePerceptionResult(res, sub_res);
    }

    return res;
}

}
