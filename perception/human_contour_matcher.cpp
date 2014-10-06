#include "human_contour_matcher.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>


/*
    This perception module can eventually be dynamically loaded into ED. For now, you can test the
    module by using ED's 'test-perception' tool. This tool works as follows:

        rosrun test-perception LIRBARY_PATH MEASUREMENT_DIRECTORY

    The first argument, LIBRARY_PATH is a path to the library file (.so) of this perception module.
    As you can see in the CMakeLists.txt, this file is compiled into a seperate library called

        libhuman_contour_matcher.so

    The second argument, MEASUREMENT_DIRECTORY, specifies the directory that contains (RGBD)
    measurements the 'test-perception' tool should test your module on. Each measurement consists
    of 2 files with the same name but different extensions: .mask and .rgbd. The '.mask' file
    contains the mask, i.e., the part of the images that was segmented out by ED. The '.rgbd'
    file contains the RGBD-data. Example measurements can be found in ed_data_storage/rgbd_measurements
    (contained in data-pkgs)

    'test-perception' will crawl through the MEASUREMENT_DIRECTORY and feed all the measurements
    to your perception module, and show the outcome.

    Example:

        rosbuild:   rosrun ed test-perception `rospack find ed`/lib/libhuman_contour_matcher.so `rospack find ed_data_storage`/rgbd_measurements
        catkin:     rosrun ed test-perception ~/ros/groovy/catkin_ws/devel/lib/libhuman_contour_matcher.so `rospack find ed_data_storage`/rgbd_measurements

    And press space to go to the next measurements.

*/

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
    /*
    Load any model specific data here (e.g., in this case, the human contour templates)

        model_name: the name of the model (e.g. 'human')
        model_path: the directory in which the models are stored
    */

    if (model_name.compare("human") == 0){
        init_success_ = human_classifier_.Initializations(model_name, model_path + "/");
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

    if (classification_error > 0){
    // assert results
        result.writeGroup("matching_result");
        result.setValue("stance", classification_stance);
        result.setValue("error", classification_error);
        result.setValue("deviation", classification_deviation);
        result.endGroup();
    }

    if(is_human)
    {
        result.setValue("type", "human_contour");
        result.setValue("type-score", 1.0);
    }else{
        result.setValue("type", "human_contour");
        result.setValue("type-score", 0.0);
    }
}

ED_REGISTER_PERCEPTION_MODULE(HumanContourMatcher)
