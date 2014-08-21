#include "qr_detector.h"

#include "ed/measurement.h"

#include <rgbd/View.h>
#include "ed/mask.h"

#include "qr_detector_zbar/qr_detector_zbar.h"

// ----------------------------------------------------------------------------------------------------

void snapZHorizontalOrVertical(const geo::Pose3D& pose, geo::Pose3D& pose_snapped) {
    geo::Vector3 b1 = pose.getBasis().getColumn(0);
    geo::Vector3 b2 = pose.getBasis().getColumn(1);
    geo::Vector3 b3 = pose.getBasis().getColumn(2);

    geo::Vector3 z_MAP(0,0,1);

    if (b3.dot(z_MAP) > sqrt(.5)) {
        // Horizontal plane (constrain 2 rotations)
        b3 = z_MAP;
        b1.z = 0; b1 = b1.normalized();
        b2 = b3.cross(b1);
    } else {
        // Vertical plane (constrain 1 rotation)
        b3.z = 0; b3 = b3.normalized();
        b2 = b3.cross(b1);
    }

    geo::Matrix3 basis(b1.x,b2.x,b3.x,b1.y,b2.y,b3.y,b1.z,b2.z,b3.z);
    pose_snapped = pose;
    pose_snapped.setBasis(basis);
}

// ----------------------------------------------------------------------------------------------------

QRDetector::QRDetector() : PerceptionModule("qr_detector")
{
}

// ----------------------------------------------------------------------------------------------------

QRDetector::~QRDetector()
{
}

void QRDetector::configure(tue::Configuration config)
{
//    double threshold = config.value("threshold", 0.0).toReal();
//    std::cout << "QRDetector: threshold = " << threshold << std::endl;
}


// ----------------------------------------------------------------------------------------------------

void QRDetector::loadModel(const std::string& model_name, const std::string& model_path)
{

}

// ----------------------------------------------------------------------------------------------------

ed::PerceptionResult QRDetector::process(const ed::Measurement& msr) const
{
    ed::PerceptionResult res;

    const cv::Mat& rgb_image = msr.image()->getRGBImage();

    rgbd::View view(*msr.image(), rgb_image.cols);

    // Create the rect
    std::vector<cv::Point2i> pnts;
    for (ed::ImageMask::const_iterator it = msr.imageMask().begin(rgb_image.cols); it != msr.imageMask().end(); ++it ) {
        pnts.push_back(*it);
    }
    cv::Rect rect = cv::boundingRect(pnts);

    std::map< std::string, std::vector<cv::Point2i> > data;
    qr_detector_zbar::getQrCodes(rgb_image(rect),data);

    for ( std::map< std::string, std::vector<cv::Point2i> >::const_iterator it = data.begin(); it != data.end(); ++it )
    {
        const std::string& label = it->first;

        std::vector<geo::Vector3> v;
        for (std::vector<cv::Point2i>::const_iterator pit = it->second.begin(); pit != it->second.end(); ++pit) {
            geo::Vector3 p;
            if (view.getPoint3D(rect.x + pit->x,rect.y + pit->y,p)) {
                v.push_back(p);
            }
        }

        geo::Pose3D pose;
        if (qr_detector_zbar::getPoseFromCornerPoints(v,pose)) {
            pose = msr.sensorPose() * pose;
            geo::Pose3D snapped;
            snapZHorizontalOrVertical(pose,snapped);
            res.addInfo(label,1.0, snapped);
        }
    }

    return res;
}

ED_REGISTER_PERCEPTION_MODULE(QRDetector)
