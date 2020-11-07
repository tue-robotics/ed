#ifndef RENDERING_H
#define RENDERING_H

#include <geolib/datatypes.h>

// Forward declarations
namespace ed {
    class WorldModel;
}
namespace geo {
    class DepthCamera;
}
namespace cv {
    class Mat;
}

namespace ed
{

enum ShowVolumes
{
    NoVolumes,
    ModelVolumes,
    RoomVolumes
};

bool renderWorldModel(const ed::WorldModel& world_model, const enum ShowVolumes show_volumes,
                      const geo::DepthCamera& cam, const geo::Pose3D& cam_pose,
                      cv::Mat& depth_image, cv::Mat& image);

}  // End of namespace ed


#endif // RENDERING_H
