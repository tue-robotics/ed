#include <ed/perception/label_gui_server.h>

#include <ros/ros.h>

#include <ed/entity.h>
#include <ed/measurement.h>

#include <rgbd/Image.h>

#include <ros/package.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_label_gui_server");

    GUIServer server;

    // - - - - - - - - - - - - - - - configure - - - - - - - - - - - - - - -

    // Get the ED directory
    std::string ed_dir = ros::package::getPath("ed");

    // Load the YAML config file
    tue::Configuration config;
    config.loadFromYAMLFile(ed_dir + "/config/config.yml");

    // Configure ED
    if (config.readGroup("gui"))
    {
        server.configure(config);
        config.endGroup();
    }
    else
    {
        std::cout << "Error during configuration: group 'gui' not found" << std::endl;
        return 1;
    }

    //! TODO: This does not work yet! (broken yaml file fails somewhere else but we do not receive an error here)
    if (config.hasError())
    {
        std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


    std::map<ed::UUID, ed::EntityConstPtr> entities;

    ed::EntityPtr e1(new ed::Entity("entity-1"));
    {
        geo::DepthCamera cam_model;
        cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 255));
        cv::Mat depth_image(480, 640, CV_32FC1, 0.0);

        rgbd::ImagePtr image(new rgbd::Image(rgb_image, depth_image, cam_model, "NO_FRAME", 0));
        ed::ImageMask image_mask(rgb_image.cols, rgb_image.rows);
        for(int y = 50; y < image_mask.height() - 50; ++y)
            for(int x = 100; x < image_mask.width() - 100; ++x)
                image_mask.addPoint(x, y);


        ed::MeasurementPtr m1(new ed::Measurement(image, image_mask));
        e1->addMeasurement(m1);
    }
    entities[e1->id()] = e1;

    ed::EntityPtr e2(new ed::Entity("entity-2"));
    {
        geo::DepthCamera cam_model;
        cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(255, 0, 0));
        cv::Mat depth_image(480, 640, CV_32FC1, 0.0);

        rgbd::ImagePtr image(new rgbd::Image(rgb_image, depth_image, cam_model, "NO_FRAME", 0));
        ed::ImageMask image_mask(rgb_image.cols, rgb_image.rows);
        for(int y = 50; y < image_mask.height() - 50; ++y)
            for(int x = 100; x < image_mask.width() - 100; ++x)
                image_mask.addPoint(x, y);


        ed::MeasurementPtr m1(new ed::Measurement(image, image_mask));
        e2->addMeasurement(m1);
    }
    entities[e2->id()] = e2;

    double alpha = 0;

    ros::Rate r(10);
    while(ros::ok())
    {
        // Check if configuration has changed. If so, call reconfigure
        if (config.sync())
        {
            if (config.readGroup("gui"))
            {
                server.configure(config, true);
                config.endGroup();
            }
        }

        double x = sin(alpha) * 3 + 5;
        double y = cos(alpha) * 3 + 5;

        // Add convex hull
        ed::ConvexHull2D chull;
        chull.chull.push_back(pcl::PointXYZ(x -0.5, y -0.5, 0));
        chull.chull.push_back(pcl::PointXYZ(x + 0.5,y -0.5, 0));
        chull.chull.push_back(pcl::PointXYZ(x + 0.5,y +  0.5, 0));
        chull.chull.push_back(pcl::PointXYZ(x -0.5,y +  0.5, 0));
        chull.min_z = 0;
        chull.max_z = 1;
        e1->setConvexHull(chull);

        server.update(entities);

        alpha += 0.1;

        r.sleep();
    }

    return 0;
}
