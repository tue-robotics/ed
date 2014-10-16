#include "odu_finder_module.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include "odu_finder.h"

// ----------------------------------------------------------------------------------------------------

ODUFinderModule::ODUFinderModule() : ed::PerceptionModule("odu_finder")
{
}

// ----------------------------------------------------------------------------------------------------

ODUFinderModule::~ODUFinderModule()
{
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::loadConfig(const std::string& config_path)
{
    config_path_ = config_path;

    odu_finder_ = new odu_finder::ODUFinder();
    odu_finder_->load_database(config_path_);
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->bestMeasurement();
    if (!msr)
        return;

    // ----------------------- PREPARE IMAGE -----------------------

    cv::Mat masked_mono_image;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

    cv::Mat mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);
    // create a mask in CV Mat
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        mask.at<unsigned char>(*it) = 255;
    }

    // improve the contours of the mask
    OptimizeContourHull(mask, mask);

    // apply the mask to the whole image
    cropped_image.copyTo(masked_mono_image, mask);

    // convert it to grayscale
    cv::cvtColor(masked_mono_image, masked_mono_image, CV_BGR2GRAY);


    // ----------------------- PROCESS IMAGE -----------------------

    IplImage img(masked_mono_image);
    std::map<std::string, float> results = odu_finder_->process_image(&img);


    // ----------------------- SAVE RESULTS -----------------------


    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("odu_finder");

    // if an hypothesis is found, assert it
    if (!results.empty())
    {
        result.writeArray("hypothesis");
        for(std::map<std::string, float>::const_iterator it = results.begin(); it != results.end(); ++it)
        {
            result.addArrayItem();
            result.setValue("name", it->first);
            result.setValue("score", it->second);
            result.endArrayItem();
        }
        result.endArray();
    }

    result.endGroup();  // close odu_finder group
    result.endGroup();  // close perception_result group
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::OptimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const{

    std::vector<std::vector<cv::Point> > hull;
    std::vector<std::vector<cv::Point> > contours;

    cv::findContours(mask_orig, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (uint i = 0; i < contours.size(); i++){
        hull.push_back(std::vector<cv::Point>());
        cv::convexHull(cv::Mat(contours[i]), hull.back(), false);

        mask_optimized = cv::Mat::zeros(mask_orig.size(), CV_8UC1);

        cv::drawContours(mask_optimized, hull, -1, cv::Scalar(255), CV_FILLED);
    }
}

ED_REGISTER_PERCEPTION_MODULE(ODUFinderModule)
