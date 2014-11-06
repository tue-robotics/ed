#ifndef ED_PERCEPTION_COLOR_MATCHER_H_
#define ED_PERCEPTION_COLOR_MATCHER_H_

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>

// Color name table
#include "color_name_table.h"

#include <ed/perception_modules/perception_module.h>

using namespace ColorNames;


class ColorMatcher : public ed::PerceptionModule
{

public:

    ColorMatcher();

    virtual ~ColorMatcher();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

private:

    ColorNameTable& color_table_;


    // module configuration
    bool init_success_;
    bool kDebugMode;
    std::string kDebugFolder;
    std::string kModuleName;

    // Module methods
    std::map<std::string, double> getImageColorProbability(const cv::Mat &img, const cv::Mat &mask) const;
    std::string getHighestProbColor(std::map<std::string, double>& map) const;

    // Object colors
    std::map<std::string, std::map<std::string, double> > object_colors_;

    std::string GenerateID() const;

    void CleanDebugFolder(const std::string& folder) const;

    void optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const;
};


#endif

