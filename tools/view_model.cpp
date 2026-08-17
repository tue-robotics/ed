#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ed/entity.h>
#include <ed/models/model_loader.h>
#include <ed/rendering.h>
#include <ed/update_request.h>
#include <ed/world_model.h>

#include <geolib/datatypes.h>
#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h> // IWYU pragma: keep -- geo::Shape must be complete for visual()->getMesh()

#include <iostream>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ed/types.h"
#include <ostream>
#include <sstream>
#include <string>

#include <vector>

namespace
{

constexpr double CANVAS_WIDTH = 800;
constexpr double CANVAS_HEIGHT = 600;

// M_PI_2 is a POSIX macro from <math.h>, which C++ does not guarantee via <cmath>.
constexpr double HALF_PI = 1.5707963267948966;

/// Mutable state of the viewer, shared between main() and the mouse callback.
struct ViewerState
{
    geo::DepthCamera cam;

    geo::Vector3 cam_lookat;
    double cam_dist{0};
    double cam_yaw{0};
    double cam_pitch{0};
    cv::Point last_mouse_pos;
    bool do_rotate{true};
    geo::Pose3D cam_pose;

    bool do_flyto{false};
    geo::Vector3 cam_lookat_flyto;

    bool render_required{true};

    cv::Mat depth_image;
    cv::Mat image;
};

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: ed_view_model [ --file | --model ] FILE-OR-MODEL-NAME" << '\n';
}

// ----------------------------------------------------------------------------------------------------

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
    ViewerState& state = *static_cast<ViewerState*>(userdata);

    if (event == cv::EVENT_LBUTTONDBLCLK)
    {
        float const d = state.depth_image.at<float>(y, x);
        if (d > 0)
        {
            state.cam_lookat_flyto = state.cam_pose * (state.cam.project2Dto3D(x, y) * d);
            state.do_flyto = true;
        }
    }
    else if (event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN || event == cv::EVENT_MBUTTONDOWN)
    {
        state.last_mouse_pos = cv::Point(x, y);
        state.do_rotate = false;
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
        double const dx = x - state.last_mouse_pos.x;
        double const dy = y - state.last_mouse_pos.y;

        if (flags & cv::EVENT_FLAG_LBUTTON)
        {
            state.cam_yaw -= dx * 0.003;
            state.cam_pitch += dy * 0.003;

            if (state.cam_pitch > 1.57)
                state.cam_pitch = 1.57;
            else if (state.cam_pitch < -1.57)
                state.cam_pitch = -1.57;
        }
        else if (flags & cv::EVENT_FLAG_MBUTTON)
        {
            state.cam_dist += state.cam_dist * dy * 0.003;
        }
        else if (flags & cv::EVENT_FLAG_RBUTTON)
        {
            state.cam_lookat += state.cam_pose.R * (geo::Vector3(-dx, dy, 0) * 0.001 * state.cam_dist);
        }

        state.last_mouse_pos = cv::Point(x, y);
    }
}

} // namespace

// ----------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        usage();
        return 1;
    }

    std::string const load_type_str = argv[1];
    if (load_type_str != "--model" && load_type_str != "--file")
    {
        std::cerr << "Load type should either be --model or --file" << '\n';
        usage();
        return 1;
    }
    ed::models::LoadType const load_type =
        (load_type_str == "--model") ? ed::models::LoadType::MODEL : ed::models::LoadType::FILE;
    std::string const source = argv[2];

    ed::UpdateRequest req;
    if (!ed::models::loadModel(load_type, source, req))
        return 1;

    // Create world
    ed::WorldModel world_model;
    world_model.update(req);

    ViewerState state;

    // Set camera specs
    state.cam = geo::DepthCamera(CANVAS_WIDTH,
                                 CANVAS_HEIGHT,
                                 0.87 * CANVAS_WIDTH,
                                 0.87 * CANVAS_WIDTH,
                                 (CANVAS_WIDTH / 2) + 0.5,
                                 (CANVAS_HEIGHT / 2) + 0.5,
                                 0,
                                 0);

    // Determine min and max coordinates of model
    geo::Vector3 p_min(1e9, 1e9, 1e9);
    geo::Vector3 p_max(-1e9, -1e9, -1e9);

    std::size_t n_vertices = 0;
    std::size_t n_triangles = 0;

    for (const auto& e : world_model)
    {
        if (e->visual())
        {
            const std::string& id = e->id().str();
            if (id.size() < 5 || id.substr(id.size() - 5) != "floor") // Filter ground plane
            {
                const std::vector<geo::Vector3>& vertices = e->visual()->getMesh().getPoints();
                for (const auto& vertice : vertices)
                {
                    const geo::Vector3& p = e->pose() * vertice;
                    p_min.x = std::min(p.x, p_min.x);
                    p_min.y = std::min(p.y, p_min.y);
                    p_min.z = std::min(p.z, p_min.z);

                    p_max.x = std::max(p.x, p_max.x);
                    p_max.y = std::max(p.y, p_max.y);
                    p_max.z = std::max(p.z, p_max.z);
                }
            }

            n_vertices += e->visual()->getMesh().getPoints().size();
            n_triangles += e->visual()->getMesh().getTriangleIs().size();
        }
    }

    double const dist = 2 * std::max({p_max.z - p_min.z, p_max.x - p_min.x, p_max.y - p_min.y});

    std::stringstream info_msg;
    info_msg << "Model loaded successfully:" << '\n';
    info_msg << "    " << n_vertices << " vertices" << '\n';
    info_msg << "    " << n_triangles << " triangles" << '\n';
    info_msg << "    " << "x: [" << p_min.x << " - " << p_max.x << "]" << '\n';
    info_msg << "    " << "y: [" << p_min.y << " - " << p_max.y << "]" << '\n';
    info_msg << "    " << "z: [" << p_min.z << " - " << p_max.z << "]" << '\n';

    info_msg << '\n';
    info_msg << "Mouse:" << '\n';
    info_msg << "    left         - orbit" << '\n';
    info_msg << "    middle       - zoom" << '\n';
    info_msg << "    right        - pan" << '\n';
    info_msg << "    double click - fly to" << '\n';

    info_msg << '\n';
    info_msg << "Keys:" << '\n';
    info_msg << "    r - reload model" << '\n';
    info_msg << "    v - hide all volumes, show model volumes, show room volumes" << '\n';
    info_msg << "    c - circle rotate" << '\n';
    info_msg << "    p - snap pitch" << '\n';
    info_msg << "    q - quit" << '\n';

    std::cout << info_msg.str();

    ed::ShowVolumes show_volumes = ed::ModelVolumes;

    state.cam_dist = dist;
    state.cam_lookat = (p_min + p_max) / 2;
    state.cam_yaw = 0;
    state.cam_pitch = 0.7;

    // Create a window
    cv::namedWindow("visualization", 1);

    // set the callback function for any mouse event
    cv::setMouseCallback("visualization", mouseCallback, &state);

    while (true)
    {
        const geo::Pose3D old_cam_pose = state.cam_pose;
        state.cam_pose.t =
            geo::Vector3(cos(state.cam_yaw), sin(state.cam_yaw), 0) * cos(state.cam_pitch) * state.cam_dist;
        state.cam_pose.t.z = sin(state.cam_pitch) * state.cam_dist;
        state.cam_pose.t += state.cam_lookat;

        geo::Vector3 const rz = -(state.cam_lookat - state.cam_pose.t).normalized();
        geo::Vector3 const rx = geo::Vector3(0, 0, 1).cross(rz).normalized();
        geo::Vector3 const ry = rz.cross(rx).normalized();

        state.cam_pose.R = geo::Matrix3(rx, ry, rz);

        if (!state.render_required && old_cam_pose != state.cam_pose)
        {
            state.render_required = true;
        }

        if (state.render_required)
        {
            state.depth_image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);
            state.image = cv::Mat(state.depth_image.rows,
                                  state.depth_image.cols,
                                  CV_8UC3,
                                  cv::Scalar(20, 20, 20)); // Not completely black
            ed::renderWorldModel(
                world_model, show_volumes, state.cam, state.cam_pose.inverse(), state.depth_image, state.image);
            state.render_required = false;
        }

        cv::imshow("visualization", state.image);
        int const key = cv::waitKey(10);

        if (key == 'r')
        {
            ed::UpdateRequest req;
            if (ed::models::loadModel(load_type, source, req))
            {
                world_model = ed::WorldModel();
                world_model.update(req);
            }
            state.render_required = true;
        }
        else if (key == 'v')
        {
            show_volumes = ed::ShowVolumes((show_volumes + 1) % 3);
            state.render_required = true;
        }
        else if (key == 'q')
        {
            break;
        }
        else if (key == 'c')
        {
            state.do_rotate = !state.do_rotate;
        }
        else if (key == 'p')
        {
            // Snap pitch to 90 degrees
            if (state.cam_pitch < HALF_PI)
                state.cam_pitch = std::round((state.cam_pitch / HALF_PI) + 0.51) * HALF_PI;
            else
                state.cam_pitch = std::round((state.cam_pitch / HALF_PI) - 0.51) * HALF_PI;

            state.render_required = true;
        }

        if (state.do_rotate)
        {
            state.cam_yaw += 0.03;
            state.render_required = true;
        }

        if (state.do_flyto)
        {
            geo::Vector3 const diff = state.cam_lookat_flyto - state.cam_lookat;
            double const dist = diff.length();

            double const max_dist = std::max(0.001 * state.cam_dist, dist * 0.1);
            if (dist < max_dist)
            {
                state.cam_lookat = state.cam_lookat_flyto;
                state.do_flyto = false;
            }
            else
            {
                state.cam_lookat += (diff / dist) * max_dist;
            }
        }
    }

    return 0;
}
