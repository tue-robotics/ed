// ROS
#include <ros/console.h>

// TU/e Robotics
#include <geolib/sensors/DepthCamera.h>
#include <geolib/Mesh.h>
#include <geolib/Box.h>
#include <tue/config/reader.h>

// ED
#include "ed/entity.h"
#include "ed/rendering.h"
#include "ed/world_model.h"

namespace ed {


float COLORS[27][3] = { { 0.6, 0.6, 0.6}, { 0.6, 0.6, 0.4}, { 0.6, 0.6, 0.2},
                        { 0.6, 0.4, 0.6}, { 0.6, 0.4, 0.4}, { 0.6, 0.4, 0.2},
                        { 0.6, 0.2, 0.6}, { 0.6, 0.2, 0.4}, { 0.6, 0.2, 0.2},
                        { 0.4, 0.6, 0.6}, { 0.4, 0.6, 0.4}, { 0.4, 0.6, 0.2},
                        { 0.4, 0.4, 0.6}, { 0.4, 0.4, 0.4}, { 0.4, 0.4, 0.2},
                        { 0.4, 0.2, 0.6}, { 0.4, 0.2, 0.4}, { 0.4, 0.2, 0.2},
                        { 0.2, 0.6, 0.6}, { 0.2, 0.6, 0.4}, { 0.2, 0.6, 0.2},
                        { 0.2, 0.4, 0.6}, { 0.2, 0.4, 0.4}, { 0.2, 0.4, 0.2},
                        { 0.2, 0.2, 0.6}, { 0.2, 0.2, 0.4}, { 0.2, 0.2, 0.2} };


class SampleRenderResult : public geo::RenderResult
{

public:

    SampleRenderResult(cv::Mat& z_buffer, cv::Mat& image)
        : geo::RenderResult(z_buffer.cols, z_buffer.rows), z_buffer_(z_buffer), image_(image)
    {
    }

    void setMesh(const geo::Mesh* mesh)
    {
        mesh_ = mesh;
        vals_.resize(mesh_->getTriangleIs().size());
        vals_.assign(mesh_->getTriangleIs().size(), -1);
    }

    inline void setColor(const cv::Vec3b& color) { color_ = color; }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer_.at<float>(y, x);
        if (old_depth == 0. || depth < old_depth)
        {
            z_buffer_.at<float>(y, x) = depth;

            if (vals_[i_triangle] < 0)
            {
                // calculate normal
                geo::Vec3 n = mesh_->getTriangleNormal(i_triangle);

                vals_[i_triangle] = (1 + n.dot(geo::Vec3(0, 0.3, -1).normalized())) / 2;
            }

            image_.at<cv::Vec3b>(y, x) = vals_[i_triangle] * color_;
        }
    }

protected:

    cv::Mat& z_buffer_;
    cv::Mat& image_;
    const geo::Mesh* mesh_;
    cv::Vec3b color_;
    std::vector<double> vals_;

};


unsigned int djb2(const std::string& str)
{
    int hash = 5381;
    for(unsigned int i = 0; i < str.size(); ++i)
        hash = ((hash << 5) + hash) + str[i]; /* hash * 33 + c */

    if (hash < 0)
        hash = -hash;

    return hash;
}


// Might it be nicer to separate rendering of the colored image and the depth image?
bool renderWorldModel(const ed::WorldModel& world_model, const enum ShowVolumes show_volumes,
                      const geo::DepthCamera& cam, const geo::Pose3D& cam_pose,
                      cv::Mat& depth_image, cv::Mat& image)
{

    if (depth_image.rows != image.rows || depth_image.cols != image.cols)
    {
       throw std::invalid_argument("Depth image and image must be of the same size");
    }

    SampleRenderResult res(depth_image, image);

    // Draw axis

    double al = 0.25; // axis length (m)
    double at = 0.01; // axis thickness (m)

    geo::Mesh x_box = geo::Box(geo::Vector3(0, -at, -at), geo::Vector3(al, at, at)).getMesh();
    geo::Mesh y_box = geo::Box(geo::Vector3(-at, 0, -at), geo::Vector3(at, al, at)).getMesh();
    geo::Mesh z_box = geo::Box(geo::Vector3(-at, -at, 0), geo::Vector3(at, at, al)).getMesh();

    geo::RenderOptions opt;

    res.setColor(cv::Vec3b(0, 0, 255));
    res.setMesh(&x_box);
    opt.setMesh(x_box, cam_pose.inverse());
    cam.render(opt, res);

    res.setColor(cv::Vec3b(0, 255, 0));
    res.setMesh(&y_box);
    opt.setMesh(y_box, cam_pose.inverse());
    cam.render(opt, res);

    res.setColor(cv::Vec3b(255, 0, 0));
    res.setMesh(&z_box);
    opt.setMesh(z_box, cam_pose.inverse());
    cam.render(opt, res);

    for(ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        const std::string& id = e->id().str();

        if (e->shape() && e->has_pose() && (id.size() < 5 || id.substr(id.size() - 5) != "floor")) // Filter ground plane
        {

            if (show_volumes == RoomVolumes && (id.size() < 4 || id.substr(0, 4) != "wall")) continue;

            tue::config::Reader config(e->data());
            if (config.readGroup("color"))
            {
                double r, g, b;
                if (config.value("red", r) && config.value("green", g) && config.value("blue", b))
                    res.setColor(cv::Vec3b(255 * b, 255 * g, 255 * r));
                config.endGroup();
            }
            else
            {
                int i_color = djb2(id) % 27;
                res.setColor(cv::Vec3b(255 * COLORS[i_color][2], 255 * COLORS[i_color][1], 255 * COLORS[i_color][0]));
            }

            res.setMesh(&e->shape()->getMesh());

            geo::Pose3D pose = cam_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            // Render
            cam.render(opt, res);

            // Render volumes
            if (show_volumes == ModelVolumes && !e->volumes().empty())
            {
                for (std::map<std::string, geo::ShapeConstPtr>::const_iterator it = e->volumes().begin(); it != e->volumes().end(); ++it)
                {
                res.setColor(cv::Vec3b(0, 0, 255)); // Red
                res.setMesh(&it->second->getMesh());
                opt.setMesh(it->second->getMesh(), pose);
                cam.render(opt, res);
                }
            }
        }
        else if (show_volumes == RoomVolumes && e->types().find("room") != e->types().end())
        {
            geo::Pose3D pose = cam_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            for (std::map<std::string, geo::ShapeConstPtr>::const_iterator it = e->volumes().begin(); it != e->volumes().end(); ++it)
            {
                res.setColor(cv::Vec3b(0, 0, 255)); // Red
                res.setMesh(&it->second->getMesh());
                opt.setMesh(it->second->getMesh(), pose);
                cam.render(opt, res);
            }
        }

    }
    return true;
}

}
