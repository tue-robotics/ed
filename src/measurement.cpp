#include "ed/measurement.h"
#include "ed/mask.h"
#include "ed/rgbd_data.h"

#include <geolib/datatypes.h>
#include <rgbd/types.h>

#include <utility>
#include <vector>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement() : timestamp_(0) {}

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement(const rgbd::ImageConstPtr& image, ImageMask image_mask, const geo::Pose3D& sensor_pose) :
    image_mask_(std::move(image_mask)), timestamp_(image->getTimestamp())
{
    rgbd_data_.image = image;
    rgbd_data_.sensor_pose = sensor_pose;
}

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement(const RGBDData& rgbd_data, PointCloudMaskPtr mask, unsigned int seq) :
    rgbd_data_(rgbd_data), mask_(std::move(mask)), timestamp_(rgbd_data.image->getTimestamp()), seq_(seq)
{
    // Calculate image mask
    image_mask_.setSize(rgbd_data.image->getDepthImage().cols, rgbd_data.image->getDepthImage().rows);
    for (int const it : *mask_)
    {
        const std::vector<int>& pixel_idxs = rgbd_data_.point_cloud_to_pixels_mapping[it];
        for (int const pixel_idx : pixel_idxs)
            image_mask_.addPoint(pixel_idx);
    }
}

} // namespace ed
