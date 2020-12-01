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

    SampleRenderResult(cv::Mat& z_buffer_, cv::Mat& image_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_), image(image_)
    {
    }

    void setMesh(const geo::Mesh* mesh_)
    {
        mesh = mesh_;
        vals.resize(mesh->getTriangleIs().size());
        vals.assign(mesh->getTriangleIs().size(), -1);
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0. || depth < old_depth)
        {
            z_buffer.at<float>(y, x) = depth;

            if (vals[i_triangle] < 0)
            {
                const geo::TriangleI& t = mesh->getTriangleIs()[i_triangle];
                const geo::Vector3& p1 = mesh->getPoints()[t.i1_];
                const geo::Vector3& p2 = mesh->getPoints()[t.i2_];
                const geo::Vector3& p3 = mesh->getPoints()[t.i3_];

                // calculate normal
                geo::Vec3 n = ((p3 - p1).cross(p2 - p1)).normalized();

                vals[i_triangle] = (1 + n.dot(geo::Vec3(0, 0.3, -1).normalized())) / 2;
            }

            image.at<cv::Vec3b>(y, x) = vals[i_triangle] * color;
        }
    }

    cv::Mat& z_buffer;
    cv::Mat& image;
    const geo::Mesh* mesh;
    cv::Vec3b color;
    std::vector<double> vals;

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

    res.color = cv::Vec3b(0, 0, 255);
    res.setMesh(&x_box);
    opt.setMesh(x_box, cam_pose.inverse());
    cam.render(opt, res);

    res.color = cv::Vec3b(0, 255, 0);
    res.setMesh(&y_box);
    opt.setMesh(y_box, cam_pose.inverse());
    cam.render(opt, res);

    res.color = cv::Vec3b(255, 0, 0);
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
                res.color = cv::Vec3b(255 * b, 255 * g, 255 * r);
                config.endGroup();
            }
            else
            {
                int i_color = djb2(id) % 27;
                res.color = cv::Vec3b(255 * COLORS[i_color][2], 255 * COLORS[i_color][1], 255 * COLORS[i_color][0]);
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
                res.color = cv::Vec3b(0, 0, 255);
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
                res.color = cv::Vec3b(0, 0, 255);
                res.setMesh(&it->second->getMesh());
                opt.setMesh(it->second->getMesh(), pose);
                cam.render(opt, res);
            }
        }

    }
    return true;
}

}
