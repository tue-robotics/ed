#include "size_matcher.h"

#include "ed/measurement.h"
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
    std::string filename = model_path + "/sizes.txt";

    // Load file
    std::ifstream model_file;
    model_file.open(filename.c_str());
    if (!model_file.is_open())
    {
        std::cout << "Could not open file " << filename << std::endl;
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
}

// ----------------------------------------------------------------------------------------------------

PerceptionResult SizeMatcher::process(const Measurement& msr) const
{
    PerceptionResult res;

    const cv::Mat& rgb_image = msr.image()->getRGBImage();

    geo::Vector3 min(1e10, 1e10, 1e10);
    geo::Vector3 max(-1e10, -1e10, -1e10);

    rgbd::View view(*msr.image(), msr.imageMask().width());

    for(ed::ImageMask::const_iterator it = msr.imageMask().begin(view.getWidth()); it != msr.imageMask().end(); ++it)
    {
        const cv::Point2i& p_2d = *it;

        geo::Vector3 p;
        if (view.getPoint3D(p_2d.x, p_2d.y, p))
        {
            geo::Vector3 p_MAP = msr.sensorPose() * p;
            min.x = std::min(min.x, p_MAP.x); max.x = std::max(max.x, p_MAP.x);
            min.y = std::min(min.y, p_MAP.y); max.y = std::max(max.y, p_MAP.y);
            min.z = std::min(min.z, p_MAP.z); max.z = std::max(max.z, p_MAP.z);
        }
    }

    geo::Vector3 size = max - min;
    double width = sqrt(size.x * size.x + size.z * size.z);
    double height = size.y;

//    std::cout << "SizeMatcher: width = " << width << ", height = " << height << std::endl << std::endl;

    std::vector<std::string> hyps;

    for(std::map<std::string, std::vector<ObjectSize> >::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& label = it->first;
        const std::vector<ObjectSize>& sizes = it->second;

        bool match = false;
        for(std::vector<ObjectSize>::const_iterator it_size = sizes.begin(); it_size != sizes.end(); ++it_size)
        {
            const ObjectSize& size = *it_size;
//            std::cout << "    " << size.min_width << " " << size.max_width << std::endl;
            if (height >= size.min_height && height <= size.max_height &&
                    width >= size.min_width && width <= size.max_width)
            {
                match = true;
                break;
            }
        }

        if (match)
            hyps.push_back(label);
        else
            res.addInfo(label, 0);
    }

    if (!hyps.empty())
    {
        double prob = 1.0 / hyps.size();
        for(std::vector<std::string>::const_iterator it = hyps.begin(); it != hyps.end(); ++it)
            res.addInfo(*it, prob);
    }

    return res;
}

ED_REGISTER_PERCEPTION_MODULE(SizeMatcher)

}

}
