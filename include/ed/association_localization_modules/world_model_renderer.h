#ifndef _world_model_renderer_H_
#define _world_model_renderer_H_

#include "ed/types.h"
#include <geolib/sensors/DepthCamera.h>
#include <pcl/point_types.h>

#include <rgbd/View.h>

namespace ed
{

class WorldModelRenderer
{

public:

    WorldModelRenderer();

    virtual ~WorldModelRenderer();

    void render(const geo::Pose3D& camera_pose,
                const std::map<UUID, EntityConstPtr>& entities,
                float max_range,
                const rgbd::View& view,
                cv::Mat& img,
                pcl::PointCloud<pcl::PointXYZ>& pc,
                std::vector<const Entity*>& pc_entity_ptrs);


private:

    geo::DepthCamera camera_model_;

};

}

#endif
