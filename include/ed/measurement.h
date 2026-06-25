#ifndef measurement_h_
#define measurement_h_

#include "ed/mask.h"
#include "ed/rgbd_data.h"
#include "ed/types.h"

namespace ed
{

class Measurement
{

public:
    Measurement();

    Measurement(const rgbd::ImageConstPtr& image, ImageMask image_mask, const geo::Pose3D& sensor_pose);

    Measurement(const RGBDData& rgbd_data, PointCloudMaskPtr mask, unsigned int seq = 0);

    [[nodiscard]]
    const geo::Pose3D& sensorPose() const
    {
        return rgbd_data_.sensor_pose;
    }
    [[nodiscard]]
    rgbd::ImageConstPtr image() const
    {
        return rgbd_data_.image;
    }
    [[nodiscard]]
    PointCloudMaskConstPtr mask() const
    {
        return mask_;
    }
    [[nodiscard]]
    const ImageMask& imageMask() const
    {
        return image_mask_;
    }
    [[nodiscard]]
    double timestamp() const
    {
        return timestamp_;
    }

protected:
    RGBDData rgbd_data_;
    PointCloudMaskPtr mask_;
    ImageMask image_mask_;
    double timestamp_;
    unsigned int seq_{};
};

} // namespace ed

#endif
