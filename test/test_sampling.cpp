#include "rgbd/Client.h"
#include "rgbd/View.h"

#include <opencv2/highgui/highgui.hpp>
#include "ed/helpers/depth_data_processing.h"
#include <ed/helpers/visualization.h>
#include <ed/measurement.h>

#include <profiling/Timer.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_sampling");

    rgbd::Client client;
    client.intialize("/amigo/top_kinect/rgbd");

    int width = 640;
    int height = 480;

    ed::ImageMask mask(width, height);
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            mask.addPoint(x, y);
        }
    }

    ros::Rate r(30);
    while (ros::ok()) {
        rgbd::ImageConstPtr image = client.nextImage();
        if (image) {
            rgbd::View view(*image, width);

            ed::IndexMap indices;

            Timer timer;
            timer.start();
//            pcl::PointCloud<pcl::PointXYZ>::Ptr pc = ed::helpers::ddp::getPclFromDepthImageMask(image, mask, 0.03, 10, 8, indices);

            ed::RGBDData rgbd_data;
            rgbd_data.image = image;

            ed::helpers::ddp::extractPointCloud(rgbd_data, 0.03, 3, 4);

            timer.stop();
            std::cout << "Sampling took " << timer.getElapsedTimeInMilliSec() << " ms" << std::endl;

            std::cout << "Sampled to " << rgbd_data.point_cloud->points.size() << " points" << std::endl;

            ed::PointCloudMaskPtr pc_mask(new ed::PointCloudMask(rgbd_data.point_cloud->points.size()));
            for(unsigned int i = 0; i < pc_mask->size(); ++i)
                (*pc_mask)[i] = i;

            ed::MeasurementPtr m(new ed::Measurement(rgbd_data, pc_mask));

            cv::Mat img(height, width, CV_32FC1, 0.0);
            for(unsigned int i = 0; i < rgbd_data.point_cloud->points.size(); ++i)
            {
                const pcl::PointXYZ& p = rgbd_data.point_cloud->points[i];
                cv::Point2i p_2d = view.getRasterizer().project3Dto2D(geo::Vector3(p.x, p.y, p.z));
                img.at<float>(p_2d) = 1;
            }

            cv::imshow("Sampled points", img);
            cv::waitKey(3);

            ed::helpers::visualization::showMeasurement(m, "test_sampling");
        }

        r.sleep();
    }

    return 0;
}
