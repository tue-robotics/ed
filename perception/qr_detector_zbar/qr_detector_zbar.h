#ifndef qr_detector_zbar_H_
#define qr_detector_zbar_H_

#include <opencv/cv.h>
#include <geolib/datatypes.h>
#include <rgbd/Image.h>

namespace qr_detector_zbar {

void getQrCodes(const cv::Mat& rgb_image, std::map<std::string, std::vector<cv::Point2i> >& data);

bool getPoseFromCornerPoints(const std::vector<geo::Vector3> v, geo::Pose3D& pose);

}

#endif
