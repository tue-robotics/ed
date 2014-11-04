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

    // creat odu finder instance
    odu_finder_ = new odu_finder::ODUFinder(config_path_, false);
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
    uint min_x, max_x, min_y, max_y;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

    // initialize bounding box points
    max_x = 0;
    max_y = 0;
    min_x = view.getWidth();
    min_y = view.getHeight();

    // create a mask in CV Mat
    cv::Mat mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        mask.at<unsigned char>(*it) = 255;

        // update the boundary coordinates
        if (min_x > p_2d.x) min_x = p_2d.x;
        if (max_x < p_2d.x) max_x = p_2d.x;
        if (min_y > p_2d.y) min_y = p_2d.y;
        if (max_y < p_2d.y) max_y = p_2d.y;
    }

    cv::cvtColor(cropped_image, masked_mono_image, CV_BGR2GRAY);


    // ----------------------- PROCESS IMAGE -----------------------

    IplImage img(masked_mono_image(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)));
    std::map<std::string, float> results;

    {
        boost::lock_guard<boost::mutex> lg(mutex_update_);
        results = odu_finder_->process_image(&img);
    }


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
            result.setValue("name", it->first + "_odu");
            result.setValue("score", it->second);
            result.endArrayItem();
        }
        result.endArray();
    }

    result.endGroup();  // close odu_finder group
    result.endGroup();  // close perception_result group
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::OptimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized, cv::Rect& bounding_box) const{

    std::vector<std::vector<cv::Point> > hull;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Rect> bounding_boxes;
    cv::Mat tempMat;

    cv::findContours(mask_orig, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    tempMat = cv::Mat::zeros(mask_orig.size(), CV_8UC1);

    for (uint i = 0; i < contours.size(); i++){
        hull.push_back(std::vector<cv::Point>());
        cv::convexHull(cv::Mat(contours[i]), hull.back(), false);

        bounding_boxes.push_back(cv::boundingRect(hull.back()));

        cv::drawContours(tempMat, hull, -1, cv::Scalar(255), CV_FILLED);
    }

//    cv::imwrite("/tmp/odu/opt.png", mask_optimized);

    contours.clear();
    mask_optimized = cv::Mat(tempMat);

    cv::findContours(tempMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (contours.size()>0) bounding_box = cv::boundingRect(contours[0]);

//    cv::imwrite("/tmp/odu/bounding.png", mask_optimized(bounding_box));
}

ED_REGISTER_PERCEPTION_MODULE(ODUFinderModule)
