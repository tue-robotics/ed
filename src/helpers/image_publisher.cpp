#include "ed/helpers/image_publisher.h"

#include <ros/node_handle.h>
#include <rgbd/serialization.h>
#include <rgbd/RGBDMsg.h>

#include <tue/serialization/conversions.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

ImagePublisher::ImagePublisher()
{
}

// ----------------------------------------------------------------------------------------------------

ImagePublisher::~ImagePublisher()
{
}

// ----------------------------------------------------------------------------------------------------

void ImagePublisher::initialize(const char* name)
{
    ros::NodeHandle nh("~");
    pub_ = nh.advertise<rgbd::RGBDMsg>(name, 1);
}

// ----------------------------------------------------------------------------------------------------

void ImagePublisher::publish(const cv::Mat& img) const
{
    if (!enabled())
        return;

    // TODO: this should be so much easier and more efficient!

    geo::DepthCamera cam;
    cv::Mat depth_image;

    rgbd::Image image(img, depth_image, cam, "", 0);

    std::stringstream stream;
    tue::serialization::OutputArchive a(stream);
    rgbd::serialize(image, a, rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_NONE);

    rgbd::RGBDMsg msg;
    msg.version = 2;
    tue::serialization::convert(stream, msg.rgb);

    pub_.publish(msg);
}

// ----------------------------------------------------------------------------------------------------

bool ImagePublisher::enabled() const
{
    return (initialized() && pub_.getNumSubscribers() > 0);
}

// ----------------------------------------------------------------------------------------------------

bool ImagePublisher::initialized() const
{
    return (!pub_.getTopic().empty());
}

} // end namespace ed

