#include "odu_finder_module.h"

#include "ed/measurement.h"
#include "ed/entity.h"

#include <rgbd/Image.h>

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
//    odu_finder_ = new odu_finder::ODUFinder();

//    std::string config_path_copy = config_path;
//    odu_finder_->load_database(config_path_copy);

    config_path_ = config_path;
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->bestMeasurement();
    if (!msr)
        return;

    odu_finder::ODUFinder odu_finder_;
    odu_finder_.load_database(config_path_);

    // get the RGB image of the measurement
    const cv::Mat& rgb_image = msr->image()->getRGBImage();

    // Convert to grayscale
    cv::Mat mono_image;
    cv::cvtColor(rgb_image, mono_image, CV_BGR2GRAY);

    // apply the measurment mask to the image
    cv::Mat masked_mono_image(mono_image.rows, mono_image.cols, mono_image.type(), cv::Scalar(0));
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(mono_image.cols); it != msr->imageMask().end(); ++it)
        masked_mono_image.at<unsigned char>(*it) = mono_image.at<unsigned char>(*it);

    // process the image
    IplImage img(masked_mono_image);
    std::map<std::string, float> results = odu_finder_.process_image(&img);

    result.writeArray("hypotheses");
    for(std::map<std::string, float>::const_iterator it = results.begin(); it != results.end(); ++it)
    {
        result.nextArrayItem();
        result.setValue("type", it->first);
        result.setValue("score", it->second);
    }
    result.endArray();
}

ED_REGISTER_PERCEPTION_MODULE(ODUFinderModule)
