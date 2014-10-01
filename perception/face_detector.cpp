#include "face_detector.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>


// ----------------------------------------------------------------------------------------------------

FaceDetector::FaceDetector() :
    PerceptionModule("face_detector"),
    init_success_(false)
{

}


// ----------------------------------------------------------------------------------------------------

FaceDetector::~FaceDetector()
{
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::loadModel(const std::string& model_name, const std::string& model_path)
{
    /*
    Load any model specific data here
        model_name: the name of the model (e.g. 'human')
        model_path: the directory in which the models are stored
    */

    std::cout << "loadModel " << model_name << std::endl;

    if (model_name.compare("face") == 0){

//        Initializations(model_name, "face", model_path);
        kModuleName = "face_detector";
        kModelName = model_name;
        kCascadePath = model_path + "cascade_classifiers/";
        init_success_ = true;
    }
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    if (!init_success_)
        return;

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->bestMeasurement();
    if (!msr)
        return;

    // Get the depth image from the measurement
    const cv::Mat& depth_image = msr->image()->getDepthImage();
    const cv::Mat& color_image = msr->image()->getRGBImage();

    cv::Mat mask_cv = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_8UC1);

    // Iterate over all points in the mask
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(depth_image.cols); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        mask_cv.at<unsigned char>(p_2d) = 255;
    }

    // - - - - -

    // Initialize cascade classifiers
    cv::CascadeClassifier classifierFront;
    cv::CascadeClassifier classifierProfile;

    // load training files, if it fails bail out
    if (!LoadCascades(classifierFront, classifierProfile)){
        result.setValue("type", "face");
        result.setValue("type-score", 0.0);
        return;
    }


    result.setValue("type", "face");
    result.setValue("type-score", 1.0);
}

// ----------------------------------------------------------------------------------------------------

//void FaceDetector::Initializations( const std::string& module_name,
//                                    const std::string& model_name,
//                                    const std::string& module_path) const
//{
//    kModuleName = module_name;
//    kModelName = model_name;
//    kCascadePath = module_path + "cascade_classifiers/";
//}

// ----------------------------------------------------------------------------------------------------

bool FaceDetector::LoadCascades(cv::CascadeClassifier classifierFront,
                                cv::CascadeClassifier classifierProfile) const
{
    if (!classifierFront.load(kCascadePath + "haarcascade_frontalface_default.xml") ||
            !classifierProfile.load(kCascadePath + "haarcascade_profileface.xml")) {

        std::cout << "[" << kModuleName << "] " << "Unable to load all haar cascade files ("<< kCascadePath << ")" << std::endl;
        return false;
    } else {
        std::cout << "[" << kModuleName << "] " << "Haar cascade XML files sucessfully loaded." << std::endl;
    }
}


ED_REGISTER_PERCEPTION_MODULE(FaceDetector)
