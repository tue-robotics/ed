#include <rgbd/Image.h>
#include <ed/measurement.h>

#include <ed/io/filesystem/read.h>

#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {

    if (argc != 2)
    {
        std::cout << "Usage:\n\n   view FILENAME\n\n";
        return 1;
    }

    std::string filename = argv[1];

    // load measurement
    ed::Measurement msr;
    if (ed::read(filename, msr))
    {
        const cv::Mat& rgb_image = msr.image()->getRGBImage();

        cv::Mat masked_image(rgb_image.rows, rgb_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

        for(ed::ImageMask::const_iterator it = msr.imageMask().begin(rgb_image.cols); it != msr.imageMask().end(); ++it)
        {
            masked_image.at<cv::Vec3b>(*it) = rgb_image.at<cv::Vec3b>(*it);
        }

        cv::imshow("TestPerceptionModule: RGB", masked_image);
        cv::waitKey();
    }
    else
    {
        std::cout << "Could not load measurement." << std::endl;
        return 1;
    }

    return 0;
}
