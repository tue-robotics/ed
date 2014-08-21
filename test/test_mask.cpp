#include <ed/mask.h>

#include <profiling/Timer.h>

int main(int argc, char **argv) {

    cv::Mat rgb_image(480, 640, CV_8UC3, cv::Scalar(1, 0, 255));

    ed::ImageMask m(rgb_image.cols, rgb_image.rows);

    for(int y = 0; y < 480; ++y)
    {
        for(int x = 0; x < 640; ++x)
        {
            m.addPoint(x, y);
        }
    }

//    cv::Point2i p_min, p_max;
//    m.boundingRect(p_min, p_max);
//    std::cout << p_min << " - " << p_max << std::endl;

    int N = 1;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    {
        Timer timer;
        timer.start();

        int i = 0;
        for(int n = 0; n < N; ++n)
        {
            for(int y = 0; y < rgb_image.rows; ++y)
            {
                for(int x = 0; x < rgb_image.cols; ++x)
                {
                    i += rgb_image.at<cv::Vec3b>(y, x)[0];
                }
            }
        }

        timer.stop();

        std::cout << "Check value: " << i << std::endl;
        std::cout << timer.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    {
        Timer timer;
        timer.start();

        int i = 0;
        for(int n = 0; n < N; ++n)
        {
            for(ed::ImageMask::const_iterator it = m.begin(); it != m.end(); ++it)
            {
//                std::cout << *it << std::endl;
                i += rgb_image.at<cv::Vec3b>(*it)[0];
            }
        }

        timer.stop();

        std::cout << "Check value: " << i << std::endl;
        std::cout << timer.getElapsedTimeInMilliSec() / N << " ms" << std::endl;

    }

    return 0;
}
