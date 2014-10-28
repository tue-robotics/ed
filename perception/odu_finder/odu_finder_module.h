#ifndef ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_
#define ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_

#include <ed/perception_modules/perception_module.h>
#include <boost/thread.hpp>

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

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

private:

    std::string config_path_;

    void OptimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized, cv::Rect &bounding_box) const;

    odu_finder::ODUFinder* odu_finder_;

protected:

    mutable boost::mutex mutex_update_;


};

#endif
