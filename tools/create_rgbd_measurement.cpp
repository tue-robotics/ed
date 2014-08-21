#include <ed/io/filesystem/write.h>
#include <ed/measurement.h>
#include <ed/mask.h>

#include <rgbd/Image.h>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>

int main(int argc, char **argv)
{
    if( argc != 2)
    {
        std::cout << "Usage:\n\n   create-rgbd-measurement RGB_IMAGE\n\n";
        return 1;
    }

    std::string filename_rgb = argv[1];

    // Read the file
    cv::Mat rgb_image = cv::imread(filename_rgb, CV_LOAD_IMAGE_COLOR);

    // Check for invalid input
    if (!rgb_image.data )
    {
        std::cout <<  "Could not open or find the image." << std::endl ;
        return 1;
    }

    cv::Mat depth_image(rgb_image.rows, rgb_image.cols, CV_32FC1, cv::Scalar(0.0));

    geo::DepthCamera cam_model;
    cam_model.setFocalLengths(500, 500);
    cam_model.setOpticalCenter(320.5, 480.5);
    cam_model.setOpticalTranslation(0, 0);

    rgbd::ImagePtr image(new rgbd::Image(rgb_image, depth_image, cam_model, "", 0));

    cv::imshow("rgb", image->getRGBImage());
    cv::waitKey();

    ed::ImageMask mask(rgb_image.cols, rgb_image.rows);
    for(int y = 0; y < mask.height(); ++y)
        for(int x = 0; x < mask.width(); ++x)
            mask.addPoint(x, y);

//    ed::Measurement msr(geo::Pose3D::identity(), image, mask);

//    ed::write(filename_rgb, msr);

    return 0;
}
