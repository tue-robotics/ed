#ifndef ED_PERCEPTION_FACE_DETECTOR_H_
#define ED_PERCEPTION_FACE_DETECTOR_H_

#include <ed/perception_modules/perception_module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

class FaceDetector : public ed::PerceptionModule
{

/*
 * ###########################################
 *  				PRIVATE
 * ###########################################
 */
private:

    // module configuration
    bool init_success_;
    bool kDebugMode;            /*!< Enable debug mode */
    std::string	kModuleName;    /*!< Name of the module, for output */
    std::string	kModelName;     /*!< Name of the classified model */
    std::string kDebugFolder;   /*!< Path of the debug folder */
    std::string kCascadePath;   /*!< Path of the cascade training folder */


    // Cascade classifier configuration
    double kClassFrontScaleFactor;  // Parameter specifying how much the image size is reduced at each image scale
    int kClassFrontMinNeighbors;    // Parameter specifying how many neighbors each candidate rectangle should have to retain it.
    cv::Size kClassFrontMinSize;    // Minimum possible object size. Objects smaller than that are ignored.

    double kClassProfileScaleFactor;  // Parameter specifying how much the image size is reduced at each image scale
    int kClassProfileMinNeighbors;    // Parameter specifying how many neighbors each candidate rectangle should have to retain it.
    cv::Size kClassProfileMinSize;   // Minimum possible object size. Objects smaller than that are ignored.

    mutable cv::CascadeClassifier classifier_front;
    mutable cv::CascadeClassifier classifier_profile;

    bool DetectFaces(const cv::Mat &masked_img,
                     const cv::Mat &mask_cv,
                     const std::string &entity_id,
                     std::vector<cv::Rect>& faces_front,
                     std::vector<cv::Rect>& faces_profile) const;

    void CleanDebugFolder(const std::string& folder);

    int ClipInt(int val, int min, int max) const;

    void OptimizeContour(const cv::Mat& mask_orig, const cv::Mat& mask_opt) const;


public:

    FaceDetector();

    virtual ~FaceDetector();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

};

#endif
