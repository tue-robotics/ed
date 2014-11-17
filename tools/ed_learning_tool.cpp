#include <ed/perception_modules/perception_module.h>

#include <ed/entity.h>

// Measurement data structures
#include <ed/measurement.h>
#include <rgbd/Image.h>

// File crawling
#include <tue/filesystem/crawler.h>

// Model loading
#include <ros/package.h>

// Measurement loading
#include <ed/io/filesystem/read.h>
#include <fstream>
#include <rgbd/serialization.h>
#include <ed/serialization/serialization.h>

// Show measurement
#include <opencv2/highgui/highgui.hpp>

#include "odu_finder_learner.cpp"

// RGBD
#include <rgbd/Image.h>
#include <rgbd/View.h>

// file writting
#include <fstream>
#include <string>
#include <iostream>

std::string kModuleName;

// ----------------------------------------------------------------------------------------------------

void showMeasurement(const ed::Measurement& msr)
{
    const cv::Mat& rgb_image = msr.image()->getRGBImage();
    const cv::Mat& depth_image = msr.image()->getDepthImage();

    cv::Mat masked_rgb_image(rgb_image.rows, rgb_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat masked_depth_image(depth_image.rows, depth_image.cols, depth_image.type(), 0.0);

    for(ed::ImageMask::const_iterator it = msr.imageMask().begin(rgb_image.cols); it != msr.imageMask().end(); ++it)
        masked_rgb_image.at<cv::Vec3b>(*it) = rgb_image.at<cv::Vec3b>(*it);

    for(ed::ImageMask::const_iterator it = msr.imageMask().begin(depth_image.cols); it != msr.imageMask().end(); ++it)
        masked_depth_image.at<float>(*it) = depth_image.at<float>(*it);

    cv::imshow("Measurement: depth", masked_depth_image / 8);
    cv::imshow("Measurement: rgb", masked_rgb_image);
}

// ----------------------------------------------------------------------------------------------------

void config_to_file(tue::Configuration& config, const std::string &model_name, const std::string &save_directory){
    std::string filename = save_directory + "/" + model_name + ".yaml";

    std::cout << "[" << kModuleName << "] " << "Saving config for '" << model_name << "' at " << filename << std::endl;
//    std::cout << "[" << kModuleName << "] \n" << config << std::endl;

    std::ofstream out(filename.c_str(), std::ofstream::out);
    if (out.is_open()){
        out << config.toYAMLString();
        out.close();
    }else
        std::cout << "[" << kModuleName << "]" << "Could not create file" << std::endl;
}


// ----------------------------------------------------------------------------------------------------

void parse_config(tue::Configuration& config, const std::string &module_name, const std::string &model_name, tue::Configuration& final_config){

    // --------------- PARSE INFORMATION ---------------

    // step into perception_result group
    if (!config.readGroup("perception_result")){
        std::cout << "[" << kModuleName << "] " << "Could not find the perception_result group" << std::endl;
        return;
    }

    // step into the group being parsed
    if (!config.readGroup(module_name)){
        std::cout << "[" << kModuleName << "] " << "Could not find the " << module_name << "group" << std::endl;
        config.endGroup(); // close type_aggregator group in case this one fails
        return;
    }

    float height = 0;
    float width = 0;
    std::string color_name;
    float amount;
    std::map<std::string, float> color_info;

    std::cout << "[" << kModuleName << "] " << "Parsing " << module_name << " group" << std::endl;

    // parse information from current config
    if(module_name.compare("size_matcher") == 0){
        if (config.readGroup("size")){
            if (config.value("height", height) && config.value("width", width)){
//                std::cout << "[" << kModuleName << "] " << "Read HxW " << height << " x " << width << std::endl;
            }
            config.endGroup(); // close size group
        }
    }else if(module_name.compare("color_matcher") == 0){
        if (config.readArray("hypothesis")){
            while(config.nextArrayItem()){
                if (config.value("name", color_name) && config.value("score", amount)){
//                    std::cout << "[" << kModuleName << "] " << "Read color: " << color_name << " (" << amount << ")" << std::endl;
                    color_info[color_name] = amount;
                }
            }
            config.endArray(); // close hypothesis array
        }
    }

    config.endGroup(); // close the parsed group
    config.endGroup(); // close perception_result group


    // --------------- WRITE PARSED INFORMATION ---------------


    // create or read perception_result group
    if (!final_config.readGroup("model", tue::OPTIONAL)){
        final_config.writeGroup("model");
    }

    final_config.setValue("name", model_name);

    // save size information if it was parsed
    if (height > 0 && width > 0){
        if (!final_config.readArray("size", tue::OPTIONAL)){
            final_config.writeArray("size");
        }

        final_config.addArrayItem();
        final_config.setValue("height", height);
        final_config.setValue("width", width);
        final_config.endArrayItem();

        final_config.endArray();
    }

    // save color information if it was parsed
    if(!color_info.empty()){
        if (!final_config.readArray("color", tue::OPTIONAL)){
            final_config.writeArray("color");
        }

        for(std::map<std::string, float>::const_iterator color_it = color_info.begin(); color_it != color_info.end(); ++color_it) {
            final_config.addArrayItem();
            final_config.setValue("name", color_it->first);
            final_config.setValue("amount", color_it->second);
            final_config.endArrayItem();
        }

        final_config.endArray();
    }

    final_config.endGroup(); // close model group
}

// ----------------------------------------------------------------------------------------------------

void optimizeContourBlur(const cv::Mat& mask_orig, cv::Mat& mask_optimized){

    mask_orig.copyTo(mask_optimized);

    // blur the contour, also expands it a bit
    for (uint i = 6; i < 18; i = i + 2){
        cv::blur(mask_optimized, mask_optimized, cv::Size( i, i ), cv::Point(-1,-1) );
    }

    cv::threshold(mask_optimized, mask_optimized, 50, 255, CV_THRESH_BINARY);
}

// ----------------------------------------------------------------------------------------------------


void imageToOduFinder(ed::EntityPtr& entity, OduFinderLearner& odu_learner){
    // ---------- PREPARE MEASUREMENT ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = entity->bestMeasurement();
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

//    cv::imwrite("/tmp/learner/" + entity->id() + "_learner.png", roi);

    odu_learner.learnImage(entity->id(), roi);
}


// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv) {

    std::string measurement_dir;
    std::string output_dir;

    if (argc < 3 || argc > 3)
    {
        std::cout << "Usage for:\n\n   ed-learning-tool MEASUREMENT_DIRECTORY OUTPUT_DIRECTORY \n\n";
        return 1;
    }else if (argc == 3)
    {
        measurement_dir = argv[1];
        output_dir = argv[2];
    }

    std::vector<std::string> perception_libs;
    kModuleName = "ed_learning_tool";

    // used plugins
    OduFinderLearner odu_learner = OduFinderLearner();
    perception_libs.push_back("/home/luisf/ros/hydro/dev/devel/lib/libsize_matcher.so");
    perception_libs.push_back("/home/luisf/ros/hydro/dev/devel/lib/libcolor_matcher.so");


    // ---------------- LOAD PERCEPTION LIBRARIES ----------------

    std::vector<ed::PerceptionModulePtr> modules;

    std::vector<class_loader::ClassLoader*> perception_loaders(perception_libs.size(), 0);
    for(unsigned int i = 0; i < perception_libs.size(); ++i)
    {
        class_loader::ClassLoader* class_loader = new class_loader::ClassLoader(perception_libs[i]);
        perception_loaders[i] = class_loader;

        ed::PerceptionModulePtr perception_mod = ed::loadPerceptionModule(class_loader);
        if (perception_mod)
        {
            modules.push_back(perception_mod);
        }
        else
        {
            std::cout << "Unable to load perception module " << perception_libs[i] << std::endl;
        }
    }

    // ---------------- CRAWL THROUGH MEASUREMENTS ----------------

    tue::filesystem::Crawler crawler(measurement_dir);

    tue::Configuration parsed_conf;
    int n_measurements = 0;
    tue::filesystem::Path filename;
    std::string model_name;
    std::string last_model = "";
    std::string parent_dir;
    bool first_model = true;

    while(crawler.nextPath(filename))
    {
        if (filename.extension() != ".mask")
            continue;

        // load measurement onto an dummy entity
        ed::MeasurementPtr msr(new ed::Measurement);
        if (!ed::read(filename.withoutExtension().string(), *msr))
        {
            continue;
        }

        // get info on model name from path
        parent_dir = filename.withoutExtension().string().substr(0, filename.withoutExtension().string().find_last_of("/")-1);
        model_name = parent_dir;      // create copy of the path
        model_name = model_name.substr(1, model_name.find_last_of("/") - 1);    // remove last folder name
        model_name = model_name.substr(model_name.find_last_of("/") + 1);    // remove last folder name

        // get the name of the first model, when found
        if(first_model){
            last_model = model_name;
            first_model = false;
        }

        // if the model name changes, save the learned information
        if (last_model.compare(model_name) != 0){
            config_to_file(parsed_conf, last_model, output_dir);
            // reset config for new model
            parsed_conf = tue::Configuration();
            last_model = model_name;
        }

        ed::EntityPtr e(new ed::Entity(model_name + "-entity", "", 5, 0));
        e->addMeasurement(msr);

        std::cout << "[" << kModuleName << "] " << "Entity '" << model_name << "' on " << filename.withoutExtension() << std::endl;

        // ---------------- PROCESS MEASUREMENTS WITH LIBRARIES----------------
        for(std::vector<ed::PerceptionModulePtr>::iterator it_mod = modules.begin(); it_mod != modules.end(); ++it_mod)
        {
            tue::Configuration entity_conf;
            (*it_mod)->process(e, entity_conf);
            parse_config(entity_conf, (*it_mod)->name(), model_name, parsed_conf);
        }

        // send the object image to OduFinder
        imageToOduFinder(e, odu_learner);

        ++n_measurements;

        //showMeasurement(*msr);
        //cv::waitKey();
    }

    // save last parsed model
    if (n_measurements == 0)
        std::cout << "No measurements found." << std::endl;
    else{
        config_to_file(parsed_conf, model_name, output_dir);
        odu_learner.build_database(output_dir);
    }


    // ---------------- KILL PERCEPTION MODULES ----------------


    // Delete all perception modules
    for(std::vector<ed::PerceptionModulePtr>::iterator it_mod = modules.begin(); it_mod != modules.end(); ++it_mod)
        it_mod->reset();

    // Delete the class loaders (which will unload the libraries)
    for(unsigned int i = 0; i < perception_loaders.size(); ++i)
        delete perception_loaders[i];

    return 0;
}

