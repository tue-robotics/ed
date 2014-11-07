#include "size_matcher.h"

#include "ed/measurement.h"
#include "ed/entity.h"
#include <rgbd/Image.h>
#include <rgbd/View.h>

// Loading models
#include <fstream>

namespace ed
{
namespace perception
{

// ----------------------------------------------------------------------------------------------------

SizeMatcher::SizeMatcher() : PerceptionModule("size_matcher")
{
}

// ----------------------------------------------------------------------------------------------------

SizeMatcher::~SizeMatcher()
{
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::loadModel(const std::string& model_name, const std::string& model_path)
{
    kModuleName = "size_matcher";

    std::string filename = model_path + "/sizes.txt";

//    std::cout << "[" << kModuleName << "] " << "Loading sizes file from " << model_path + "/sizes.txt" << std::endl;

    // Load file
    std::ifstream model_file;
    model_file.open(filename.c_str());
    if (!model_file.is_open())
    {
        std::cout << "[" << kModuleName << "] " << "Could not open file " << filename << std::endl;
        return;
    }

    std::vector<ObjectSize> model_vec;

    // Get data from file
    double h_min = 0, h_max = 0, w_min = 0, w_max = 0;
    while (model_file >> h_min >> h_max >> w_min >> w_max)
    {
        ObjectSize obj_sz(h_min, h_max, w_min, w_max);
        model_vec.push_back(obj_sz);
    }

    models_[model_name] = model_vec;

    small_tresh = 0.5;
    medium_tresh = 0.7;
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->bestMeasurement();
    if (!msr)
        return;

    const cv::Mat& rgb_image = msr->image()->getRGBImage();

    // initialize min and max coordinates
    geo::Vector3 min(1e10, 1e10, 1e10);
    geo::Vector3 max(-1e10, -1e10, -1e10);

    rgbd::View view(*msr->image(), msr->imageMask().width());

    // get minimun and maximum X, Y and Z coordinates of the object
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        const cv::Point2i& p_2d = *it;

        geo::Vector3 p;
        if (view.getPoint3D(p_2d.x, p_2d.y, p))
        {
//            geo::Vector3 p_MAP = msr->sensorPose() * p;
            geo::Vector3 p_MAP = p;     // use this only when sensorPose is not available

            min.x = std::min(min.x, p_MAP.x); max.x = std::max(max.x, p_MAP.x);
            min.y = std::min(min.y, p_MAP.y); max.y = std::max(max.y, p_MAP.y);
            min.z = std::min(min.z, p_MAP.z); max.z = std::max(max.z, p_MAP.z);
        }
    }

    // determine size
    geo::Vector3 size = max - min;
    double object_width = sqrt(size.x * size.x + size.z * size.z);
    double object_height = size.y;

    std::vector<std::string> hyps;

    // compare object size to loaded models
    for(std::map<std::string, std::vector<ObjectSize> >::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& label = it->first;
        const std::vector<ObjectSize>& sizes = it->second;

        bool match = false;
        for(std::vector<ObjectSize>::const_iterator it_size = sizes.begin(); it_size != sizes.end(); ++it_size)
        {
            const ObjectSize& model_size = *it_size;

            // if the object size is within the min and max, add hypothesis
            if (object_height >= model_size.min_height && object_height <= model_size.max_height &&
                    object_width >= model_size.min_width && object_width <= model_size.max_width)
            {
                match = true;
                break;
            }
        }

        if (match)
            hyps.push_back(label);
    }

    // ----------------------- SAVE RESULTS -----------------------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("size_matcher");

    result.writeGroup("size");
    result.setValue("width", object_width);
    result.setValue("height", object_height);
    result.endGroup();

    if ((object_width + object_height) < small_tresh){
        result.setValue("label", "small_size");
    }else if (small_tresh < (object_width + object_height) && (object_width + object_height) < medium_tresh){
        result.setValue("label", "medium_size");
    }else if (medium_tresh < (object_width + object_height)){
        result.setValue("label", "large_size");
    }

    result.setValue("score", 1.0);

    // if an hypothesis is found, assert it
    if (!hyps.empty())
    {
        // probability depends on the number of hypothesis
        double prob = 1.0 / hyps.size();

        result.writeArray("hypothesis");
        for(std::vector<std::string>::const_iterator it = hyps.begin(); it != hyps.end(); ++it)
        {
            result.addArrayItem();
            result.setValue("name", *it + "_size");
            result.setValue("score", prob);
            result.endArrayItem();
        }
        result.endArray();
    }

    result.endGroup();  // close size_matcher group
    result.endGroup();  // close perception_result group
}

ED_REGISTER_PERCEPTION_MODULE(SizeMatcher)
}
}
