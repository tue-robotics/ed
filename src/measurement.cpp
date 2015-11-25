#include "ed/measurement.h"

#include <rgbd/Image.h>

#include "ed/helpers/depth_data_processing.h"

namespace ed
{

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement() : timestamp_(0)
{
}

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement(rgbd::ImageConstPtr image, const ImageMask& image_mask, const geo::Pose3D& sensor_pose) :
    image_mask_(image_mask),
    timestamp_(image->getTimestamp())
{
    rgbd_data_.image = image;
    rgbd_data_.sensor_pose = sensor_pose;
}

// ----------------------------------------------------------------------------------------------------

Measurement::Measurement(const RGBDData& rgbd_data, const PointCloudMaskPtr& mask, unsigned int seq) :
    rgbd_data_(rgbd_data),
    mask_(mask),
    timestamp_(rgbd_data.image->getTimestamp())
{
    // Calculate image mask
    image_mask_.setSize(rgbd_data.image->getDepthImage().cols, rgbd_data.image->getDepthImage().rows);
    for(PointCloudMask::const_iterator it = mask_->begin(); it != mask_->end(); ++it)
    {
        const std::vector<int>& pixel_idxs = rgbd_data_.point_cloud_to_pixels_mapping[*it];
        for(std::vector<int>::const_iterator it2 = pixel_idxs.begin(); it2 != pixel_idxs.end(); ++it2)
            image_mask_.addPoint(*it2);
    }

}

}
