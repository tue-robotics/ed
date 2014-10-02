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

    bool init_success_;
    bool kDebugMode;            /*!< Enable debug mode */
    std::string	kModuleName;    /*!< Name of the module, for output */
    std::string	kModelName;     /*!< Name of the classified model */
    std::string kDebugFolder;   /*!< Path of the debug folder */
    std::string kCascadePath;   /*!< Path of the cascade training folder */

    bool Detect(cv::CascadeClassifier& classifierFront,
                cv::CascadeClassifier& classifierProfile,
                const cv::Mat &colorImage,
                const cv::Mat &mask_cv, const std::string &entity_id) const;

    void CleanDebugFolder(const std::string& folder);

    bool LoadCascades(cv::CascadeClassifier classifierFront, cv::CascadeClassifier classifierProfile) const;

    //void Initializations(const std::string& module_name, const std::string& model_name, const std::string &module_path) const;


public:

    FaceDetector();

    virtual ~FaceDetector();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

};

#endif
