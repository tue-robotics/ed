#ifndef map_publisher_h_
#define map_publisher_h_

#include "ed/types.h"

#include <tue/config/configuration.h>

#include <ros/publisher.h>

namespace ed
{

class MapPublisher
{

public:

    MapPublisher ();

    virtual ~MapPublisher();

    void publishMap (const std::map<UUID, EntityConstPtr>& entities);

    void configure (tue::Configuration config);

protected:

    void publishMapMsg (const cv::Mat& map);

    bool worldToMap (double wx, double wy, int& mx, int& my) const;

    void mapToWorld (unsigned int mx, unsigned int my, double& wx, double& wy) const;

    void updateMap (const EntityConstPtr& e, cv::Mat& map);

    ros::Publisher map_pub_;

    //! Tunable Params --------------

    double frequency_;

    std::string frame_id_;
    std::string topic_;

    geo::Vector3 origin_;

    double size_x_, size_y_;
    int width_, height_;

    double res_;

};

}

#endif
