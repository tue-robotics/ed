#include <ed/models/model_loader.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/entity.h>

#include <geolib/sensors/DepthCamera.h>
#include <geolib/Shape.h>

#include <opencv2/highgui/highgui.hpp>

#include <ros/init.h>
#include <ros/node_handle.h>

double CANVAS_WIDTH = 640;
double CANVAS_HEIGHT = 480;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ed_view_model");  // <- TODO: GET RID OF THIS!
    ros::NodeHandle nh;

    if (argc != 2)
    {
        std::cout << "Please specify an instance name" << std::endl;
        return 1;
    }

    std::string model_name = argv[1];

    ed::UpdateRequest req;

    ed::models::ModelLoader model_loader;
    if (!model_loader.create(model_name, model_name, req))
    {
        std::cout << "Model could not be loaded." << std::endl;
        return 1;
    }

    // Create world
    ed::WorldModel world_model;
    world_model.update(req);

    // Set camera specs
    geo::DepthCamera cam;
    cam.setFocalLengths(0.87 * CANVAS_WIDTH, 0.87 * CANVAS_WIDTH);
    cam.setOpticalCenter(CANVAS_WIDTH / 2 + 0.5, CANVAS_HEIGHT / 2 + 0.5);
    cam.setOpticalTranslation(0, 0);

    // Determine min and max coordinates of model
    geo::Vector3 p_min(1e9, 1e9, 1e9);
    geo::Vector3 p_max(-1e9, -1e9, -1e9);

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
        }
    }

    double dist = std::max(p_max.z - p_min.z, std::max(p_max.x - p_min.x, p_max.y - p_min.y));
    double h = (p_max.z - p_min.z) / 2;
    double angle = 0;

    while (ros::ok())
    {
        // * * * * * * DEPTH CAMERA * * * * * *

        cv::Mat depth_image = cv::Mat(CANVAS_HEIGHT, CANVAS_WIDTH, CV_32FC1, 0.0);

        for(ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
        {
            const ed::EntityConstPtr& e = *it;
            const std::string& id = e->id().str();

            if (e->shape() && (id.size() < 5 || id.substr(id.size() - 5) != "floor")) // Filter ground plane

            {
                cam.rasterize(*e->shape(), geo::Pose3D(0, -dist, h + dist, 0.8, 0, 0), geo::Pose3D(0, 0, 0, 0, 0, angle) * e->pose(), depth_image);
            }
        }

        for(unsigned int i = 0; i < depth_image.rows * depth_image.cols; ++i)
        {
            float& d = depth_image.at<float>(i);
            if (d > 0)
                d = 1 - (d / (dist * 2));
        }

        cv::imshow("visualization", depth_image);
        cv::waitKey(10);

        angle += 0.03;
    }

   return 0;
}
