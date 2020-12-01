#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/entity.h>
#include <ed/rendering.h>

#include <fstream>

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>
#include <geolib/Box.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/config/reader.h>
#include <tue/config/reader_writer.h>
#include "tue/config/loaders/sdf.h"
#include "tue/config/loaders/xml.h"
#include "tue/config/loaders/yaml.h"

#include <tue/filesystem/path.h>

double CANVAS_WIDTH = 800;
double CANVAS_HEIGHT = 600;

geo::DepthCamera cam;

geo::Vector3 cam_lookat;
double cam_dist, cam_yaw, cam_pitch;
cv::Point LAST_MOUSE_POS;
bool do_rotate = true;
geo::Pose3D cam_pose;

bool do_flyto = false;
geo::Vector3 cam_lookat_flyto;

cv::Mat depth_image;
cv::Mat image;

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: ed_view_model [ --file | --model ] FILE-OR-MODEL-NAME" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDBLCLK)
    {
        float d = depth_image.at<float>(y, x);
        if (d > 0)
        {
            cam_lookat_flyto = cam_pose * (cam.project2Dto3D(x, y) * d);
            do_flyto = true;
        }
    }
    else if (event == cv::EVENT_LBUTTONDOWN)
    {
        LAST_MOUSE_POS = cv::Point(x, y);
        do_rotate = false;
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        LAST_MOUSE_POS = cv::Point(x, y);
        do_rotate = false;
    }
    else if (event == cv::EVENT_MBUTTONDOWN)
    {
        LAST_MOUSE_POS = cv::Point(x, y);
        do_rotate = false;
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
        double dx = x - LAST_MOUSE_POS.x;
        double dy = y - LAST_MOUSE_POS.y;

        if (flags & cv::EVENT_FLAG_LBUTTON)
        {
            cam_yaw -= dx * 0.003;
            cam_pitch += dy * 0.003;

            if (cam_pitch > 1.57)
                cam_pitch = 1.57;
            else if (cam_pitch < -1.57)
                cam_pitch = -1.57;
        }
        else if (flags & cv::EVENT_FLAG_MBUTTON)
        {
            cam_dist += cam_dist * dy * 0.003;
        }
        else if (flags & cv::EVENT_FLAG_RBUTTON)
        {
            cam_lookat += cam_pose.R * (geo::Vector3(-dx, dy, 0) * 0.001 * cam_dist);
        }

        LAST_MOUSE_POS = cv::Point(x, y);
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        usage();
        return 1;
    }

    std::string load_type_str = argv[1];
    ed::models::LoadType load_type;
    if (load_type_str == "--model")
        load_type = ed::models::LoadType::MODEL;
    else if (load_type_str == "--file")
        load_type = ed::models::LoadType::FILE;
    else
    {
        std::cerr << "Load type should either be --model or --file" << std::endl;
        usage();
        return 1;
    }
    std::string source = argv[2];

    ed::UpdateRequest req;
    if (!ed::models::loadModel(load_type, source, req))
        return 1;

    // Create world
    ed::WorldModel world_model;
    world_model.update(req);

    // Set camera specs
    cam.setFocalLengths(0.87 * CANVAS_WIDTH, 0.87 * CANVAS_WIDTH);
    cam.setOpticalCenter(CANVAS_WIDTH / 2 + 0.5, CANVAS_HEIGHT / 2 + 0.5);
    cam.setOpticalTranslation(0, 0);

    // Determine min and max coordinates of model
    geo::Vector3 p_min(1e9, 1e9, 1e9);
    geo::Vector3 p_max(-1e9, -1e9, -1e9);

    int n_vertices = 0;
    int n_triangles = 0;

    for(ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape())
        {
            const std::vector<geo::Vector3>& vertices = e->shape()->getMesh().getPoints();
            for(unsigned int i = 0; i < vertices.size(); ++i)
            {
                const geo::Vector3& p = vertices[i];
                const std::string& id = e->id().str();

                if (id.size() < 5 || id.substr(id.size() - 5) != "floor") // Filter ground plane
//                if (p.z > 0.05) // Filter ground plane
                {
                    p_min.x = std::min(p.x, p_min.x);
                    p_min.y = std::min(p.y, p_min.y);
                    p_min.z = std::min(p.z, p_min.z);

                    p_max.x = std::max(p.x, p_max.x);
                    p_max.y = std::max(p.y, p_max.y);
                    p_max.z = std::max(p.z, p_max.z);
                }
            }

            n_vertices += e->shape()->getMesh().getPoints().size();
            n_triangles += e->shape()->getMesh().getTriangleIs().size();
        }
    }

    double dist = 2 * std::max(p_max.z - p_min.z, std::max(p_max.x - p_min.x, p_max.y - p_min.y));

    std::stringstream info_msg;
    info_msg << "Model loaded successfully:" << std::endl;
    info_msg << "    " << n_vertices << " vertices" << std::endl;
    info_msg << "    " << n_triangles << " triangles" << std::endl;

    info_msg << std::endl;
    info_msg << "Mouse:" << std::endl;
    info_msg << "    left         - orbit" << std::endl;
    info_msg << "    middle       - zoom" << std::endl;
    info_msg << "    right        - pan" << std::endl;
    info_msg << "    double click - fly to" << std::endl;

    info_msg << std::endl;
    info_msg << "Keys:" << std::endl;
    info_msg << "    r - reload model" << std::endl;
    info_msg << "    v - hide all volumes, show model volumes, show room volumes" << std::endl;
    info_msg << "    c - circle rotate" << std::endl;
    info_msg << "    p - snap pitch" << std::endl;
    info_msg << "    q - quit" << std::endl;

    std::cout << info_msg.str();

    ed::ShowVolumes show_volumes = ed::ModelVolumes;

    cam_dist = dist;
    cam_lookat = (p_min + p_max) / 2;
    cam_yaw = 0;
    cam_pitch = 0.7;

    //Create a window
    cv::namedWindow("visualization", 1);

    //set the callback function for any mouse event
    cv::setMouseCallback("visualization", CallBackFunc, NULL);

    while (true)
    {
        cam_pose.t = geo::Vector3(cos(cam_yaw), sin(cam_yaw), 0) * cos(cam_pitch) * cam_dist;
        cam_pose.t.z = sin(cam_pitch) * cam_dist;
        cam_pose.t += cam_lookat;

        geo::Vector3 rz = -(cam_lookat - cam_pose.t).normalized();
        geo::Vector3 rx = geo::Vector3(0, 0, 1).cross(rz).normalized();
        geo::Vector3 ry = rz.cross(rx).normalized();

        cam_pose.R = geo::Matrix3(rx.x, ry.x, rz.x,
                                  rx.y, ry.y, rz.y,
                                  rx.z, ry.z, rz.z);

        depth_image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);
        image = cv::Mat(depth_image.rows, depth_image.cols, CV_8UC3, cv::Scalar(20, 20, 20));
        ed::renderWorldModel(world_model, show_volumes, cam, cam_pose, depth_image, image);

        cv::imshow("visualization", image);
        char key = cv::waitKey(10);

        if (key == 'r')
        {
            ed::UpdateRequest req;
            if (ed::models::loadModel(load_type, source, req))
            {
                world_model = ed::WorldModel();
                world_model.update(req);
            }
        }
        else if (key == 'v')
        {
            show_volumes = ed::ShowVolumes((show_volumes + 1) % 3);
        }
        else if (key == 'q')
        {
            break;
        }
        else if (key == 'c')
        {
            do_rotate = !do_rotate;
        }
        else if (key == 'p')
        {
            // Snap pitch to 90 degrees
            if (cam_pitch < 1.57)
                cam_pitch = std::round(cam_pitch / 1.57 + 0.51) * 1.57;
            else
                cam_pitch = std::round(cam_pitch / 1.57 - 0.51) * 1.57;
        }

        if (do_rotate)
            cam_yaw += 0.03;

        if (do_flyto)
        {
            geo::Vector3 diff = cam_lookat_flyto - cam_lookat;
            double dist = diff.length();

            double max_dist = std::max(0.001 * cam_dist, dist * 0.1);
            if (dist < max_dist)
            {
                cam_lookat = cam_lookat_flyto;
                do_flyto = false;
            }
            else
            {
                cam_lookat += (diff / dist) * max_dist;
            }
        }
    }

   return 0;
}
