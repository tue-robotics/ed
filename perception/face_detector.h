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



    bool DetectFaces(const cv::Mat &colorImage,
                     const cv::Mat &mask_cv,
                     const std::string &entity_id,
                     std::vector<cv::Rect>& faces_front,
                     std::vector<cv::Rect>& faces_profile) const;

    void CleanDebugFolder(const std::string& folder);

    bool LoadCascades() const;

    int ClipInt(int val, int min, int max) const;

    void OptimizeContour(const cv::Mat& mask_orig, const cv::Mat& mask_opt) const;


public:

    FaceDetector();

    virtual ~FaceDetector();

    void loadModel(const std::string& model_name, const std::string& model_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

};

#endif
