#include "color_matcher.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>


ColorMatcher::ColorMatcher() :
    PerceptionModule("color_matcher"),
    color_table_(ColorNameTable::instance()),  // Init colorname table
    init_success_(false)
{
}

// ---------------------------------------------------------------------------------------------------

ColorMatcher::~ColorMatcher()
{
}

// ---------------------------------------------------------------------------------------------------

void ColorMatcher::loadModel(const std::string& model_name, const std::string& model_path)
{
    if (model_name.compare("color") == 0){
        kModuleName = "color_matcher";
        kDebugFolder = "/tmp/color_matcher/";
        kDebugMode = false;

        if (kDebugMode)
            CleanDebugFolder(kDebugFolder);

        std::cout << "[" << kModuleName << "] " << "Loading color names..." << std::endl;
        if (!color_table_.load_config(model_path + "/color_names.txt")){
            std::cout << "[" << kModuleName << "] " << "Failed loading color names from " + model_path + "/color_names.txt" << std::endl;
            return;
        }

        init_success_ = true;
        std::cout << "[" << kModuleName << "] " << "Ready!" << std::endl;
    }
}

// ---------------------------------------------------------------------------------------------------

void ColorMatcher::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    if (!init_success_)
        return;

    // ---------- PREPARE MEASUREMENT ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->bestMeasurement();
    if (!msr)
        return;

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

    // initialize mask
    cv::Mat mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);
    // Iterate over all points in the mask
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

    optimizeContourBlur(mask, mask);

    // ---------- PROCESS MEASUREMENT ----------

    // Calculate img color prob
    cv::Mat roi (cropped_image(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)));
    cv::Mat roi_mask (mask(cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y)));

    std::map<std::string, double> color_prob = getImageColorProbability(roi, roi_mask);

    // ---------- ASSERT RESULTS ----------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("color_matcher");

    // assert hypothesis
    if (!color_prob.empty()){

        result.writeArray("hypothesis");
        for (std::map<std::string, double>::const_iterator it = color_prob.begin(); it != color_prob.end(); ++it)
        {
            if (it->second > 0.15) {
                result.addArrayItem();
                result.setValue("name", it->first);
                result.setValue("score", it->second);
                result.endArrayItem();
            }
        }
        result.endArray();
    }

    result.endGroup();  // close color_matcher group
    result.endGroup();  // close perception_result group

    // ---------- DEBUG ----------

    if (kDebugMode){
        cv::Mat temp;
        std::string id = GenerateID();

        roi.copyTo(temp, roi_mask);

        cv::imwrite(kDebugFolder + id + "_color_matcher_full.png", roi);
        cv::imwrite(kDebugFolder + id + "_color_matcher_masked.png", temp);
    }
}

// ---------------------------------------------------------------------------------------------------

std::map<std::string, double> ColorMatcher::getImageColorProbability(const cv::Mat& img, const cv::Mat& mask) const
{
    std::map<std::string, unsigned int> color_count;
    uint pixel_count = 0;

    // Loop over the image
    for(int y = 0; y < img.rows; ++y) {
        for(int x = 0; x < img.cols; ++x) {

            // only use the points covered by the mask
            if (mask.at<unsigned char>(y, x) > 0){
                pixel_count ++;

                // Calculate prob distribution
                const cv::Vec3b& c = img.at<cv::Vec3b>(y, x);

                ColorNamePoint cp((float) c[2],(float) c[1],(float) c[0]);
                std::vector<ColorProbability> probs = color_table_.getProbabilities(cp);

                std::string highest_prob_name;
                float highest_prob = 0;

                for (std::vector<ColorProbability>::iterator it = probs.begin(); it != probs.end(); ++it) {

                    if (it->probability() > highest_prob) {
                        highest_prob = it->probability();
                        highest_prob_name = it->name();
                    }
                }

                // Check if the highest prob name exists in the map
                std::map<std::string, unsigned int>::iterator found_it = color_count.find(highest_prob_name);
                if (found_it == color_count.end()) // init on 1
                    color_count.insert( std::pair<std::string, unsigned int>(highest_prob_name,1) );
                else // +1
                    color_count[highest_prob_name] += 1;

                // Set the color in the image (vis purposes only)

                int r,g,b;
                colorToRGB(stringToColor(highest_prob_name),r,g,b);
//                img.at<cv::Vec3b>(y, x) = cv::Vec3b(b,g,r);       // Paint over the image for debugging
            }
        }
    }

    std::map<std::string, double> color_prob;
    for (std::map<std::string, unsigned int>::const_iterator it = color_count.begin(); it != color_count.end(); ++it) {
        color_prob.insert(std::pair<std::string, double>(it->first, (double) it->second/pixel_count));
    }

    return color_prob;
}

// ---------------------------------------------------------------------------------------------------

std::string ColorMatcher::getHighestProbColor(std::map<std::string, double>& map) const
{
    double max = 0;
    std::string max_name;
    for (std::map<std::string, double>::const_iterator it = map.begin(); it != map.end(); ++it)
    {
//        std::cout << "[" << kModuleName << "] " << "Color "<< it->first << " -> " << it->second << std::endl;

        if (it->second > max) {
            max = it->second;
            max_name = it->first;
        }
    }
    return max_name;
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized) const{

    mask_orig.copyTo(mask_optimized);

    // blur the contour, also expands it a bit
    for (uint i = 6; i < 18; i = i + 2){
        cv::blur(mask_optimized, mask_optimized, cv::Size( i, i ), cv::Point(-1,-1) );
    }

    cv::threshold(mask_optimized, mask_optimized, 50, 255, CV_THRESH_BINARY);
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::CleanDebugFolder(const std::string& folder) const{
    if (system(std::string("mkdir " + folder).c_str()) != 0){
        //printf("\nUnable to create output folder. Already created?\n");
    }
    if (system(std::string("rm " + folder + "*.png").c_str()) != 0){
        //printf("\nUnable to clean output folder \n");
    }
}

// ---------------------------------------------------------------------------------------------------

std::string ColorMatcher::GenerateID() const{
    static const char alphanum[] =
        "0123456789"
        "abcdef";

    std::string ID;
    for (int i = 0; i < 10; ++i) {
        int n = rand() / (RAND_MAX / (sizeof(alphanum) - 1) + 1);
        ID += alphanum[n];
    }

    return ID;
}

ED_REGISTER_PERCEPTION_MODULE(ColorMatcher)
