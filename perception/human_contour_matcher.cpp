#include "human_contour_matcher.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>


// ----------------------------------------------------------------------------------------------------

HumanContourMatcher::HumanContourMatcher() :
    PerceptionModule("human_contour_matcher"),
    human_classifier_("human_contour_matcher"), init_success_(false)
{
}

// ----------------------------------------------------------------------------------------------------

HumanContourMatcher::~HumanContourMatcher()
{
}

// ----------------------------------------------------------------------------------------------------

void HumanContourMatcher::loadModel(const std::string& model_name, const std::string& model_path)
{
    if (model_name.compare("human") == 0){
        kModuleName = "human_contour_matcher";

        init_success_ = human_classifier_.Initializations(model_name, model_path + "/");

        std::cout << "[" << kModuleName << "] " << "Ready!" << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

void HumanContourMatcher::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    // if initialization failed, return
    if (!init_success_)
        return;

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->bestMeasurement();
    if (!msr)
        return;

    float depth_sum = 0;
    float avg_depht;
    float classification_error = 0;
    float classification_deviation = 0;
    std::string classification_stance;
    uint point_counter = 0;
    bool is_human = false;

    // Get the depth image from the measurement
    const cv::Mat& depth_image = msr->image()->getDepthImage();
    const cv::Mat& color_image = msr->image()->getRGBImage();

    cv::Mat mask_cv = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_8UC1);

    // Iterate over all points in the mask. You must specify the width of the image on which you
    // want to apply the mask (see the begin(...) method).
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(depth_image.cols); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // TODO dont creat a Mat mask, just give human_classifier_.Classify a vector of 2D points!
        // paint a mask
        mask_cv.at<unsigned char>(p_2d) = 255;

        // calculate measurement average depth
        depth_sum += depth_image.at<float>(p_2d);
        point_counter++;
    }

    avg_depht = depth_sum/(float)point_counter;

    is_human = human_classifier_.Classify(depth_image, color_image, mask_cv, avg_depht, classification_error, classification_deviation, classification_stance);

    // ----------------------- assert results -----------------------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    // assert the results to the entity
    result.writeGroup(kModuleName);
    result.setValue("label", "human_shape");

    if (classification_error > 0){
        result.setValue("stance", classification_stance);
        result.setValue("error", classification_error);
        result.setValue("deviation", classification_deviation);
    }

    if(is_human)
    {
        result.setValue("score", 1.0);
    }else{
        result.setValue("score", 0.0);
    }

    result.endGroup();  // close human_contour_matcher group
    result.endGroup();  // close perception_result group
}

ED_REGISTER_PERCEPTION_MODULE(HumanContourMatcher)
