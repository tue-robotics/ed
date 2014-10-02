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
    if (model_name.compare("face") == 0){
        kModuleName = "face_detector";
        kModelName = model_name;
        kCascadePath = model_path + "/cascade_classifiers/";
        kDebugMode = true;
        if (kDebugMode) kDebugFolder = "/tmp/face_detector/";
        CleanDebugFolder(kDebugFolder);
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

    // Get the depth and color image from the measurement
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
        std::cout << "[" << kModuleName << "] " << "Haar Cascade XML files failed loading. Returning." << std::endl;
        result.setValue("type", "face");
        result.setValue("type-score", 0.0);
        return;
    }

//    std::vector <cv::Point> contour;
//    OptimizeContour(mask_cv, )

    if(Detect(classifierFront, classifierProfile, color_image, mask_cv, e->id())){
        result.setValue("type", "face");
        result.setValue("type-score", 1.0);
    }else{
        result.setValue("type", "face");
        result.setValue("type-score", 0.0);
    }
}

bool FaceDetector::Detect(cv::CascadeClassifier& classifierFront,
                          cv::CascadeClassifier& classifierProfile,
                          const cv::Mat& color_img,
                          const cv::Mat& mask_cv,
                          const std::string& entity_id) const{

    std::vector<cv::Rect> facesFront;
    std::vector<cv::Rect> facesProfile;
    cv::Mat cascadeImg;
    cv::Rect headArea;
    bool faceDetected;

    // create a copy of the color image, masked
    color_img.copyTo(cascadeImg, mask_cv);
    // increase contrast
    normalize(cascadeImg, cascadeImg, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    classifierFront.detectMultiScale(cascadeImg, facesFront, 1.2, 2, 0|CV_HAAR_SCALE_IMAGE);

    // only search profile faces if the frontal face detection failed
    if (facesFront.size() == 0){
        classifierProfile.detectMultiScale(cascadeImg, facesProfile, 1.1, 1, 0|CV_HAAR_SCALE_IMAGE);
    }

    std::cout << "front faces = " << facesFront.size() << ", profile faces = " << facesProfile.size() << std::endl;

    faceDetected = (facesFront.size() > 0 || facesProfile.size() > 0);

    // if debug mode is active and faces were found
//    if (kDebugMode && faceDetected){
      if (kDebugMode){
        cv::Mat debugImg;

        cascadeImg.copyTo(debugImg);

        for (uint j = 0; j < facesFront.size(); j++)
            cv::rectangle(debugImg, facesFront[j], cv::Scalar(0, 255, 0), 2, CV_AA);

        for (uint j = 0; j < facesProfile.size(); j++)
            cv::rectangle(debugImg, facesProfile[j], cv::Scalar(0, 0, 255), 2, CV_AA);

        cv::imwrite(kDebugFolder + entity_id + "_faces.png", debugImg);
    }

    return faceDetected;
}

// ----------------------------------------------------------------------------------------------------

bool FaceDetector::LoadCascades(cv::CascadeClassifier classifierFront,
                                cv::CascadeClassifier classifierProfile) const
{
    if (!classifierFront.load(kCascadePath + "haarcascade_frontalface_default.xml") ||
            !classifierProfile.load(kCascadePath + "haarcascade_profileface.xml")) {

        std::cout << "[" << kModuleName << "] " << "Unable to load all haar cascade files ("<< kCascadePath << ")" << std::endl;
        return false;
    } else {
//        if (kDebugMode) std::cout << "[" << kModuleName << "] " << "Haar cascade XML files sucessfully loaded." << std::endl;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void FaceDetector::CleanDebugFolder(const std::string& folder){
    if (system(std::string("mkdir " + folder).c_str()) != 0){
        //printf("\nUnable to create output folder. Already created?\n");
    }
    if (system(std::string("rm " + folder + "*.png").c_str()) != 0){
        //printf("\nUnable to clean output folder \n");
    }
}

ED_REGISTER_PERCEPTION_MODULE(FaceDetector)
