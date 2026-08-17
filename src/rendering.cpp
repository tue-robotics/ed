// TU/e Robotics
#include <cmath>
#include <geolib/Box.h>
#include <geolib/datatypes.h>
#include <geolib/math_types.h>
#include <geolib/Mesh.h>
#include <geolib/sensors/DepthCamera.h>
#include <map>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <stdexcept>
#include <string>
#include <tue/config/reader.h>
#include <vector>

// ED
#include "ed/entity.h"
#include "ed/rendering.h"
#include "ed/types.h"
#include "ed/world_model.h"

namespace ed
{

namespace
{
const float COLORS[27][3] = {{0.6, 0.6, 0.6}, {0.6, 0.6, 0.4}, {0.6, 0.6, 0.2}, {0.6, 0.4, 0.6}, {0.6, 0.4, 0.4},
                             {0.6, 0.4, 0.2}, {0.6, 0.2, 0.6}, {0.6, 0.2, 0.4}, {0.6, 0.2, 0.2}, {0.4, 0.6, 0.6},
                             {0.4, 0.6, 0.4}, {0.4, 0.6, 0.2}, {0.4, 0.4, 0.6}, {0.4, 0.4, 0.4}, {0.4, 0.4, 0.2},
                             {0.4, 0.2, 0.6}, {0.4, 0.2, 0.4}, {0.4, 0.2, 0.2}, {0.2, 0.6, 0.6}, {0.2, 0.6, 0.4},
                             {0.2, 0.6, 0.2}, {0.2, 0.4, 0.6}, {0.2, 0.4, 0.4}, {0.2, 0.4, 0.2}, {0.2, 0.2, 0.6},
                             {0.2, 0.2, 0.4}, {0.2, 0.2, 0.2}};

class SampleRenderResult : public geo::RenderResult
{

public:
    SampleRenderResult(cv::Mat& z_buffer, cv::Mat& image) :
        geo::RenderResult(z_buffer.cols, z_buffer.rows), z_buffer_(z_buffer), image_(image)
    {
    }

    void setMesh(const geo::Mesh* mesh)
    {
        mesh_ = mesh;
        vals_.resize(mesh_->getTriangleIs().size());
        vals_.assign(mesh_->getTriangleIs().size(), -1);
    }

    void setColor(const cv::Vec3b& color) { color_ = color; }

    void renderPixel(int x, int y, float depth, int i_triangle) override
    {
        float const old_depth = z_buffer_.at<float>(y, x);
        if (old_depth == 0. || depth < old_depth)
        {
            z_buffer_.at<float>(y, x) = depth;

            if (vals_[i_triangle] < 0)
            {
                geo::Vec3 const n = mesh_->getTriangleNormal(i_triangle);

                // Small color difference between surfaces
                vals_[i_triangle] = (1 + n.dot(geo::Vec3(0, 0.3, -1).normalized())) / 2;
            }

            image_.at<cv::Vec3b>(y, x) = vals_[i_triangle] * color_;
        }
    }

protected:
    // The render result writes into buffers owned by the caller.
    // NOLINTBEGIN(cppcoreguidelines-avoid-const-or-ref-data-members)
    cv::Mat& z_buffer_;
    cv::Mat& image_;
    // NOLINTEND(cppcoreguidelines-avoid-const-or-ref-data-members)
    const geo::Mesh* mesh_{};
    cv::Vec3b color_;
    std::vector<double> vals_;
};

unsigned int djb2(const std::string& str)
{
    int hash = 5381;
    for (char const i : str)
        hash = ((hash << 5) + hash) + i; /* hash * 33 + c */

    if (hash < 0)
        hash = -hash;

    return hash;
}

/**
 * @brief Render a mesh
 * @param cam DepthCamera to use
 * @param pose Pose of the mesh in the camera frame
 * @param mesh Mesh to render
 * @param color color vector
 * @param res Renderresult, which stores the renderer image
 * @param flatten Flatten all the meshes to the groundplane (default: false)
 */
void renderMesh(const geo::DepthCamera& cam,
                const geo::Pose3D& pose,
                const geo::Mesh& mesh,
                const cv::Vec3b& color,
                SampleRenderResult& res,
                bool flatten = false)
{
    geo::RenderOptions opt;
    res.setColor(color);
    geo::Mesh mesh_flat;
    if (!flatten)
    {
        res.setMesh(&mesh);
        opt.setMesh(mesh, pose);
    }
    else
    {
        for (auto point : mesh.getPoints())
        {
            point.z = 0;
            mesh_flat.addPoint(point);
        }
        for (auto triangle_indexes : mesh.getTriangleIs())
        {
            mesh_flat.addTriangle(triangle_indexes.i1_, triangle_indexes.i2_, triangle_indexes.i3_);
        }
        res.setMesh(&mesh_flat);
        opt.setMesh(mesh_flat, pose);
    }

    cam.render(opt, res);
}

} // namespace

// Might it be nicer to separate rendering of the colored image and the depth image?
bool renderWorldModel(const ed::WorldModel& world_model,
                      ShowVolumes show_volumes,
                      const geo::DepthCamera& cam,
                      const geo::Pose3D& cam_pose_inv,
                      cv::Mat& depth_image,
                      cv::Mat& image,
                      bool flatten)
{

    if (depth_image.rows != image.rows || depth_image.cols != image.cols)
    {
        throw std::invalid_argument("Depth image and image must be of the same size");
    }

    SampleRenderResult res(depth_image, image);
    geo::RenderOptions const opt;

    // Draw axis

    constexpr double al = 0.25; // axis length (m)
    constexpr double at = 0.01; // axis thickness (m)

    geo::Mesh const x_box = geo::Box(geo::Vector3(0, -at, -at), geo::Vector3(al, at, at)).getMesh();
    geo::Mesh const y_box = geo::Box(geo::Vector3(-at, 0, -at), geo::Vector3(at, al, at)).getMesh();
    geo::Mesh const z_box = geo::Box(geo::Vector3(-at, -at, 0), geo::Vector3(at, at, al)).getMesh();

    renderMesh(cam, cam_pose_inv, x_box, cv::Vec3b(0, 0, 255), res, flatten);
    renderMesh(cam, cam_pose_inv, y_box, cv::Vec3b(0, 255, 0), res, flatten);
    renderMesh(cam, cam_pose_inv, z_box, cv::Vec3b(255, 0, 0), res, flatten);

    for (const auto& e : world_model)
    {
        const std::string& id = e->id().str();

        if (e->visual() && e->hasPose() && !e->hasFlag("self") &&
            (id.size() < 5 || id.substr(id.size() - 5) != "floor")) // Filter ground plane
        {

            if (show_volumes == ShowVolumes::ROOM_VOLUMES && (id.size() < 4 || id.substr(0, 4) != "wall"))
                continue;

            cv::Vec3b color;

            tue::config::Reader config(e->data());
            if (config.readGroup("color"))
            {
                double r = NAN;
                double g = NAN;
                double b = NAN;
                if (config.value("red", r) && config.value("green", g) && config.value("blue", b))
                    color = cv::Vec3b(
                        static_cast<uchar>(255 * b), static_cast<uchar>(255 * g), static_cast<uchar>(255 * r));
                config.endGroup();
            }
            else
            {
                int const i_color = static_cast<int>(djb2(id) % 27);
                color = cv::Vec3b(static_cast<uchar>(255 * COLORS[i_color][2]),
                                  static_cast<uchar>(255 * COLORS[i_color][1]),
                                  static_cast<uchar>(255 * COLORS[i_color][0]));
            }

            geo::Pose3D const pose = cam_pose_inv * e->pose();
            renderMesh(cam, pose, e->visual()->getMesh(), color, res, flatten);

            // Render volumes
            if (show_volumes == ShowVolumes::MODEL_VOLUMES && !e->volumes().empty())
            {
                for (const auto& it : e->volumes())
                {
                    renderMesh(cam, pose, it.second->getMesh(), cv::Vec3b(0, 0, 255), res, flatten); // Red
                }
            }
        }
        else if (show_volumes == ShowVolumes::ROOM_VOLUMES && e->types().find("room") != e->types().end())
        {
            geo::Pose3D const pose = cam_pose_inv * e->pose();
            for (const auto& it : e->volumes())
            {
                renderMesh(cam, pose, it.second->getMesh(), cv::Vec3b(0, 0, 255), res, flatten); // Red
            }
        }
    }
    return true;
}

} // namespace ed
