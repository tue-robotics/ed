#ifndef measurement_h_
#define measurement_h_

#include "ed/types.h"
#include "ed/mask.h"
#include "ed/rgbd_data.h"

namespace ed
{

class Measurement
{

public:

    Measurement();

    Measurement(rgbd::ImageConstPtr image, const ImageMask& image_mask, const geo::Pose3D& sensor_pose);

    Measurement(const RGBDData& rgbd_data, const PointCloudMaskPtr& mask, unsigned int seq = 0);

    const geo::Pose3D& sensorPose() const { return rgbd_data_.sensor_pose; }
    rgbd::ImageConstPtr image() const { return rgbd_data_.image; }
    PointCloudMaskConstPtr mask() const { return mask_; }
    const ImageMask& imageMask() const { return image_mask_; }
    double timestamp() const { return timestamp_; }

protected:

    RGBDData rgbd_data_;
    PointCloudMaskPtr mask_;
    ImageMask image_mask_;
    double timestamp_;
    unsigned int seq_;

};

}

#endif
