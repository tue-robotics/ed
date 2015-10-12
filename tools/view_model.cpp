#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/entity.h>
#include <ed/serialization/serialization.h>
#include <ed/io/json_reader.h>

#include <fstream>

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>
#include <geolib/Box.h>

#include <opencv2/highgui/highgui.hpp>

#include <tue/config/reader.h>

#include <ros/init.h>
#include <ros/node_handle.h>

double CANVAS_WIDTH = 640;
double CANVAS_HEIGHT = 480;

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

float COLORS[27][3] = { { 0.6, 0.6, 0.6}, { 0.6, 0.6, 0.4}, { 0.6, 0.6, 0.2},
                        { 0.6, 0.4, 0.6}, { 0.6, 0.4, 0.4}, { 0.6, 0.4, 0.2},
                        { 0.6, 0.2, 0.6}, { 0.6, 0.2, 0.4}, { 0.6, 0.2, 0.2},
                        { 0.4, 0.6, 0.6}, { 0.4, 0.6, 0.4}, { 0.4, 0.6, 0.2},
                        { 0.4, 0.4, 0.6}, { 0.4, 0.4, 0.4}, { 0.4, 0.4, 0.2},
                        { 0.4, 0.2, 0.6}, { 0.4, 0.2, 0.4}, { 0.4, 0.2, 0.2},
                        { 0.2, 0.6, 0.6}, { 0.2, 0.6, 0.4}, { 0.2, 0.6, 0.2},
                        { 0.2, 0.4, 0.6}, { 0.2, 0.4, 0.4}, { 0.2, 0.4, 0.2},
                        { 0.2, 0.2, 0.6}, { 0.2, 0.2, 0.4}, { 0.2, 0.2, 0.2} };

// ----------------------------------------------------------------------------------------------------

unsigned int djb2(const std::string& str)
{
    int hash = 5381;
    for(unsigned int i = 0; i < str.size(); ++i)
        hash = ((hash << 5) + hash) + str[i]; /* hash * 33 + c */

    if (hash < 0)
        hash = -hash;

    return hash;
}

// ----------------------------------------------------------------------------------------------------

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
        if (old_depth == 0 || depth < old_depth)
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

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: ed_view_model [ --file | --model ] FILE-OR-MODEL-NAME" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

bool loadModel(const std::string& load_type, const std::string& source, ed::UpdateRequest& req)
{
    if (load_type == "--file")
    {
        std::ifstream f_in;
        f_in.open(source.c_str());

        if (!f_in.is_open())
        {
            std::cerr << "Could not open file '" << source << "'." << std::endl;
            return false;
        }

        std::stringstream buffer;
        buffer << f_in.rdbuf();
        std::string str = buffer.str();
        ed::io::JSONReader r(str.c_str());
        ed::deserialize(r, req);
    }
    else if (load_type == "--model")
    {
        ed::models::ModelLoader model_loader;
        std::stringstream error;
        if (!model_loader.create("_root", source, req, error))
        {
            std::cerr << "Model '" << source << "' could not be loaded:" << std::endl << std::endl;
            std::cerr << error.str() << std::endl;
            return false;
        }
    }
    else
    {
        std::cerr << "Unknown load type: '" << load_type << "'." << std::endl << std::endl;
        usage();
        return false;
    }

    return true;
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
            cam_dist += cam_dist * dy * 0.002;
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
//    ros::init(argc, argv, "ed_view_model");  // <- TODO: GET RID OF THIS!
//    ros::NodeHandle nh;

    if (argc != 3)
    {
        usage();
        return 1;
    }

    std::string load_type = argv[1];
    std::string source = argv[2];

    ed::UpdateRequest req;
    if (!loadModel(load_type, source, req))
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
    double h = (p_max.z - p_min.z) / 2;

    std::cout << "Model loaded successfully:" << std::endl;
    std::cout << "    " << n_vertices << " vertices" << std::endl;
    std::cout << "    " << n_triangles << " triangles" << std::endl;

    std::cout << std::endl;
    std::cout << "Mouse:" << std::endl;
    std::cout << "    left         - orbit" << std::endl;
    std::cout << "    middle       - zoom" << std::endl;
    std::cout << "    right        - pan" << std::endl;
    std::cout << "    double click - fly to" << std::endl;

    std::cout << std::endl;
    std::cout << "Keys:" << std::endl;
    std::cout << "    r - reload model" << std::endl;
    std::cout << "    a - show / hide model areas" << std::endl;
    std::cout << "    c - circle rotate" << std::endl;
    std::cout << "    q - quit" << std::endl;

    bool show_areas = true;

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

        // * * * * * * DEPTH CAMERA * * * * * *

        depth_image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);
        image = cv::Mat(depth_image.rows, depth_image.cols, CV_8UC3, cv::Scalar(20, 20, 20));

        SampleRenderResult res(depth_image, image);

        {
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
        }

        for(ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;
            const std::string& id = e->id().str();

            if (e->shape() && e->has_pose() && (id.size() < 5 || id.substr(id.size() - 5) != "floor")) // Filter ground plane
            {
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

//                cam_pose.inverse() * obj_pose

                geo::Pose3D pose = cam_pose.inverse() * e->pose();
                geo::RenderOptions opt;
                opt.setMesh(e->shape()->getMesh(), pose);

                // Render
                cam.render(opt, res);


                // Render areas
                geo::Shape shape;
                tue::config::Reader r(e->data());

                if (show_areas && r.readArray("areas"))
                {
                    while(r.nextArrayItem())
                    {
//                        std::cout << r.data() << std::endl;

                        std::string a_name;
                        if (!r.value("name", a_name))
                            continue;

//                        std::cout << a_name << std::endl;

                        if (ed::deserialize(r, "shape", shape))
                        {
//                            geo::Pose3D pose = geo::Pose3D(0, -dist, h + dist, 0.8, 0, 0).inverse() * (geo::Pose3D(0, 0, 0, 0, 0, angle) * e->pose());

                            res.color = cv::Vec3b(0, 0, 255);
                            opt.setMesh(shape.getMesh(), pose);
                            cam.render(opt, res);
                        }
                    }
                    r.endArray();
                }


            }
        }

//        for(int i = 0; i < depth_image.rows * depth_image.cols; ++i)
//        {
//            float& d = depth_image.at<float>(i);
//            if (d > 0)
//                d = 1 - (d / (dist * 2));
//        }

        cv::imshow("visualization", image);
        char key = cv::waitKey(10);

        if (key == 'r')
        {
            ed::UpdateRequest req;
            if (loadModel(load_type, source, req))
            {
                world_model = ed::WorldModel();
                world_model.update(req);
            }
        }
        else if (key == 'a')
        {
            show_areas = !show_areas;
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
//            if (cam_pitch < -0.785)
//                cam_pitch = -1.57;
//            else if (cam_pitch > 0.785)
//                cam_pitch = 1.57;
//            else
//                cam_pitch = 0;

            if (cam_pitch > 0)
                cam_pitch = (int)(cam_pitch / 1.57 + 0.5) * 1.57;
            else
                cam_pitch = (int)(cam_pitch / 1.57 - 0.5) * 1.57;

            // Snap yaw to 90 degrees
            if (cam_yaw > 0)
                cam_yaw = (int)(cam_yaw / 1.57 + 0.5) * 1.57;
            else
                cam_yaw = (int)(cam_yaw / 1.57 - 0.5) * 1.57;
        }
//        std::cout  << (int)key << std::endl;

        if (do_rotate)
            cam_yaw += 0.03;

        if (do_flyto)
        {
            geo::Vector3 diff = cam_lookat_flyto - cam_lookat;
            double dist = diff.length();

//            double max_dist = 0.02 * cam_dist;
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
