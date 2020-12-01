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

/**
 * @brief The ShowVolumes enum indicates which volumes to render
 */
enum ShowVolumes
{
    NoVolumes,
    ModelVolumes,
    RoomVolumes
};

/**
 * @brief renderWorldModel renders a world model on a depth image and a colored (3D) image
 * @param world_model world model to render
 * @param show_volumes configures which volumes to render
 * @param cam DepthCamera object used for rendering
 * @param cam_pose pose of the depth camera
 * @param depth_image depth image that will be rendered
 * @param image colored 3D image that will be rendered. N.B.: needs
 * to be of the same size as the depth image
 * @return
 */
bool renderWorldModel(const ed::WorldModel& world_model, const enum ShowVolumes show_volumes,
                      const geo::DepthCamera& cam, const geo::Pose3D& cam_pose,
                      cv::Mat& depth_image, cv::Mat& image);

}  // End of namespace ed


#endif // RENDERING_H
