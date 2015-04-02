#ifndef ED_IMAGE_PUBLISHER_H_
#define ED_IMAGE_PUBLISHER_H_

#include <ros/publisher.h>
#include <opencv2/core/core.hpp>

namespace ed
{

class ImagePublisher
{

public:

    ImagePublisher();

    ~ImagePublisher();

    void initialize(const char* name);

    void publish(const cv::Mat& img) const;

    bool enabled() const;

    bool initialized() const;

private:

    ros::Publisher pub_;

};

} // end namespace ed

#endif
