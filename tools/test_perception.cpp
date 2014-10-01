#include <ed/perception_modules/perception_module.h>
//#include <ed/perception/aggregator.h>

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

int main(int argc, char **argv) {

    std::string measurement_dir;
    std::vector<std::string> perception_libs;

    if (argc < 3)
    {
        std::cout << "Usage:\n\n   test_perception PERCEPTION_LIBRARY MEASUREMENT_DIRECTORY [PERCEPTION_LIBRARY_2] ...\n\n";
        return 1;
    }
    else if (argc == 3)
    {
        measurement_dir = argv[2];
        perception_libs.push_back(argv[1]);
    }
    else if (argc > 3)
    {
        measurement_dir = argv[1];
        for(int i = 2; i < argc; ++i)
            perception_libs.push_back(argv[i]);
    }

    // - - - - -

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

        }
    }

    tue::filesystem::Crawler crawler(measurement_dir);

    int n_measurements = 0;
    tue::filesystem::Path filename;
    while(crawler.nextPath(filename))
    {
        if (filename.extension() != ".mask")
            continue;

        std::cout << filename << std::endl;

        ed::MeasurementPtr msr(new ed::Measurement);
        if (!ed::read(filename.withoutExtension().string(), *msr))
        {
            continue;
        }

        showMeasurement(*msr);

        ed::EntityPtr e(new ed::Entity("test-entity", "", 5, 0));
        e->addMeasurement(msr);

        std::cout << std::endl << "------------------------------------------------------------" << std::endl;
        std::cout << "    " << filename.withoutExtension() << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl << std::endl;

        for(std::vector<ed::PerceptionModulePtr>::iterator it_mod = modules.begin(); it_mod != modules.end(); ++it_mod)
        {
            tue::Configuration result;
            (*it_mod)->process(e, result);

            // Display the result
            std::cout << result << std::endl;
        }

        ++n_measurements;

        cv::waitKey();
    }

    if (n_measurements == 0)
        std::cout << "No measurements found." << std::endl;

    // Delete all perception modules
    for(std::vector<ed::PerceptionModulePtr>::iterator it_mod = modules.begin(); it_mod != modules.end(); ++it_mod)
        it_mod->reset();

    // Delete the class loaders (which will unload the libraries)
    for(unsigned int i = 0; i < perception_loaders.size(); ++i)
        delete perception_loaders[i];

    return 0;
}
