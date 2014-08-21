#include "ed/segmentation_modules/floodfill_sm.h"

#include "ed/helpers/visualization.h"

#include <rgbd/Image.h>

namespace ed
{

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

FloodfillSM::FloodfillSM() : RGBDSegModule("flood_fill")
{

}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void FloodfillSM::process(const RGBDData& rgbd_data, std::vector<PointCloudMaskPtr>& segments)
{
//    ros::Time t_p = ros::Time::now();
//    //helpers::visualization::showSegmentedImage(rgbd_image,segments,"before");

//    // Copy and clear segments
//    std::vector<Mask> old_segments = segments;
//    segments.clear();

//    // Create smaller depth image
//    cv::Mat depth_image_small;
//    cv::resize(rgbd_image->getOriginalDepthImage(),depth_image_small,cv::Size(640,480));

//    // Loop over all segments and perform euclidean cluster segmentation per segment
//    for (std::vector<Mask>::const_iterator it = old_segments.begin(); it != old_segments.end(); ++it) {

//        // Create small mask
//        Mask mask_small;
//        cv::resize(*it,mask_small,depth_image_small.size());

//        std::vector<Mask> blobs;

//        ros::Time t_start = ros::Time::now();
//        findBlobs(depth_image_small, mask_small, blobs);
//        std::cout << "floodfill took " << (ros::Time::now()-t_start).toSec() << " seconds." << std::endl;

//        for (std::vector<Mask>::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
//            Mask mask_normal;
//            cv::resize(*it,mask_normal,cv::Size(rgbd_image->getWidth(),rgbd_image->getHeight()));

//            segments.push_back(mask_normal);
//        }
//    }

//    helpers::visualization::showSegmentedImage(rgbd_image,segments,"after");
//    std::cout << "total took " << (ros::Time::now()-t_p).toSec() << " seconds." << std::endl;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void FloodfillSM::findBlobs(cv::Mat& depth_image, const ImageMask& mask, std::vector<ImageMask>& blobs)
{
//    int label_count = -1; // starts with -1 and goes down :)

//    for(int y = 0; y < depth_image.rows; ++y) {
//        for(int x = 0; x < depth_image.cols; ++x) {

//            // Skip if not in mask or already labeled
//            if (mask.at<char>(y, x) == 0 || depth_image.at<float>(y, x) < 0) {
//                continue;
//            }

//            Mask blob = cv::Mat::zeros(mask.size(),mask.type());
//            if (findBlob(depth_image, mask, cv::Point(x,y), label_count, blob)) {
//                blobs.push_back(blob);
//            }

//            label_count--;
//        }
//    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

bool FloodfillSM::findBlob(cv::Mat& depth_image, const cv::Mat& mask, cv::Point p, int label, ImageMask& blob)
{
//    cv::Rect rect;

//    cv::floodFill(depth_image/*, mask*/, p, label, &rect, 0.1, 0.1);

//    unsigned int num = 0;
//    for(int i=rect.y; i < (rect.y+rect.height); i++) {

//        for(int j=rect.x; j < (rect.x+rect.width); j++) {

//            if(depth_image.at<float>(i,j) == label) {
//                blob.at<char>(i,j) = 255;
//                ++num;
//            }
//        }
//    }

//    return num > 10;
    return false;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

}
