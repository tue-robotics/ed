#include "ed/helpers/depth_data_processing.h"

#include "ed/mask.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <opencv2/highgui/highgui.hpp>

#include "clipper/clipper.hpp"

#include "ed/rgbd_data.h"

namespace ed
{

namespace helpers
{

namespace ddp
{

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloud(const std::vector<geo::Vector3>& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);

    out->resize(points.size());
    for(unsigned int i = 0; i < points.size(); ++i)
        out->points[i] = pcl::PointXYZ(points[i].x, points[i].y, points[i].z);

    return out;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>& in, const geo::Pose3D& pose)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);

    geo::Quaternion q_geo;
    pose.R.getRotation(q_geo);

    Eigen::Vector3f p(pose.t.x, pose.t.y, pose.t.z);
    Eigen::Quaternionf q(q_geo.w, q_geo.x, q_geo.y, q_geo.z);

    pcl::transformPointCloud(in, *out, p, q);

    return out;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void extractPointCloud(RGBDData& data, float cell_size, float max_distance, int scale_factor)
{
    data.point_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // Create an image view with full depth image resolution
    rgbd::View view(*data.image, data.image->getDepthImage().cols);

    cv::Mat depth_image = data.image->getDepthImage().clone();

    int width = depth_image.cols;
    int height = depth_image.rows;

    cv::Mat pixel_to_point_index = cv::Mat::zeros(height, width, CV_32S);

    float cell_size_times_fx = cell_size * view.getRasterizer().getFocalLengthX();

    for(int y = 0; y < depth_image.rows; y += scale_factor)
    {
        for(int x = 0; x < depth_image.cols; x += scale_factor)
        {
            float d = depth_image.at<float>(y, x);
            if (d > 0 && d == d && d < max_distance)
            {
                // calculate window size in pixels based on distance
                int w = cell_size_times_fx / d;

                int x_max = std::min(x + w, depth_image.cols - 1);
                int y_max = std::min(y + w, depth_image.rows - 1);

                int n_pixels = 0;
                geo::Vector3 p_total(0, 0,0 );

                data.point_cloud_to_pixels_mapping.push_back(std::vector<int>());

                for(int y2 = y; y2 <= y_max; ++y2)
                {
                    for(int x2 = x; x2 <= x_max; ++x2)
                    {
                        float d2 = depth_image.at<float>(y2, x2);

                        geo::Vector3 p;
                        if (d2 > 0 && d2 == d2 && std::abs(d - d2) < cell_size && view.getPoint3D(x2, y2, p))
                        {
                            depth_image.at<float>(y2, x2) = 0;

                            pixel_to_point_index.at<int32_t>(y2, x2) = (int)data.point_cloud->points.size() + 1;

                            p_total += p;

                            int idx = y2 * depth_image.cols + x2;
                            data.point_cloud_to_pixels_mapping.back().push_back(idx);

                            ++n_pixels;
                        }
                    }
                }

                geo::Vector3 p_avg = p_total / n_pixels;
                data.point_cloud->points.push_back( pcl::PointXYZ(p_avg.x,p_avg.y,p_avg.z) );
            }
        }
    }

    int l = 2;

//    cv::Mat viz(height, width, CV_32FC1, 0.0);

    // post processing: filling gaps in image mask
    for(int y = l; y < depth_image.rows - l; ++y)
    {
        for(int x = l; x < depth_image.cols - l; ++x)
        {
            float d = depth_image.at<float>(y, x);
            int i_point = pixel_to_point_index.at<int32_t>(y, x) - 1;
            if (i_point == -1 && d > 0 && d == d && d < max_distance)
            {
//                viz.at<float>(y, x) = 1;

                bool found = false;
                for(int y2 = y - l; !found && y2 <= y + l; ++y2)
                {
                    for(int x2 = x - l; !found && x2 <= x + l; ++x2)
                    {
                        float d2 = data.image->getDepthImage().at<float>(y2, x2);
                        int i_point2 = pixel_to_point_index.at<int32_t>(y2, x2) - 1;

                        if (i_point2 >= 0 && d2 > 0 && d2 == d2) //&& std::abs(d - d2) < cell_size)
                        {
                            int idx = y * depth_image.cols + x;
                            data.point_cloud_to_pixels_mapping[i_point2].push_back(idx);
                            found = true;
                        }
                    }
                }
//                if (found)
//                    viz.at<float>(y, x) = 0.5;
            }
        }
    }

//    for(int x = 0; x < width; ++x)
//        for(int y = 0; y < height; ++y)
//        {
//            if (pixel_to_point_index.at<int32_t>(y, x) > 0)
//                viz.at<float>(y, x) = (float)pixel_to_point_index.at<int32_t>(y, x) / data.point_cloud_to_pixels_mapping.size();
//             else
//                viz.at<float>(y, x) = 0;
//        }



//    cv::imshow("viz", viz);
//    cv::waitKey(3);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void calculatePointCloudNormals(RGBDData& output, int k_search)
{
    if (!output.point_cloud)
        return;

    output.point_cloud_with_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);

    if (output.point_cloud->points.size() < 3)
        return;

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(k_search);

    norm_est.setInputCloud(output.point_cloud);

    norm_est.compute(*output.point_cloud_with_normals);

    // Copy the points to the point_normals cloud
    pcl::copyPointCloud(*output.point_cloud, *output.point_cloud_with_normals);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void getValidMask(rgbd::ImageConstPtr rgbd_image, ImageMask& mask, double max_range)
{
    rgbd::View view(*rgbd_image, rgbd_image->getDepthImage().cols / 8);

    mask = ImageMask(view.getWidth(), view.getHeight());

    for(int y = 0; y < view.getHeight(); ++y) {
        for(int x = 0; x < view.getWidth(); ++x) {

            float d = view.getDepth(x,y);
            if (d > 0 && d < max_range) {
                mask.addPoint(x, y);
            }

        }
    }
}



// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

pcl::PointCloud<pcl::PointXYZ>::Ptr getPclFromDepthImageMask(rgbd::ImageConstPtr rgbd_image, const ImageMask& mask,
                                                             float cell_size, float max_distance, int scale_factor, IndexMap& indices)
{
    rgbd::View view(*rgbd_image, mask.width());

    cv::Mat masked_depth_image;
    getDepthImageFromMask(view,mask,masked_depth_image);
//    cv::imshow("sensor", masked_depth_image);
//    cv::waitKey(3);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    float cell_size_times_fx = cell_size * view.getRasterizer().getFocalLengthX();

    for(int y = 0; y < masked_depth_image.rows; y += scale_factor)
    {
        for(int x = 0; x < masked_depth_image.cols; x += scale_factor)
        {
            float d = masked_depth_image.at<float>(y, x);
            if (d > 0 && d == d && d < max_distance)
            {
                // calculate window size in pixels based on distance
                int w = cell_size_times_fx / d;

                int x_max = std::min(x + w, masked_depth_image.cols - 1);
                int y_max = std::min(y + w, masked_depth_image.rows - 1);

                int n_pixels = 0;
                geo::Vector3 p_total(0,0,0);

                indices.push_back(std::vector<cv::Point2i>());

                for(int y2 = y; y2 <= y_max; ++y2)
                {
                    for(int x2 = x; x2 <= x_max; ++x2)
                    {
                        float d2 = masked_depth_image.at<float>(y2, x2);

                        geo::Vector3 p;
                        if (d2 > 0 && d2 == d2 && std::abs(d - d2) < cell_size && view.getPoint3D(x2, y2, p))
                        {
                            masked_depth_image.at<float>(y2, x2) = 0;

                            p_total += p;

                            indices.back().push_back(cv::Point2i(x2, y2));

                            ++n_pixels;
                        }
                    }
                }

                if (n_pixels > 0)//w * w / 4)
                {
                    geo::Vector3 p_avg = p_total / n_pixels;
                    point_cloud->points.push_back( pcl::PointXYZ(p_avg.x,p_avg.y,p_avg.z) );
                }
                else
                {
                    indices.pop_back();
                }

            }
        }
    }

    return point_cloud;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void getDepthImageFromMask(const rgbd::View& view, const ImageMask& mask, cv::Mat& masked_depth_image)
{
    masked_depth_image = cv::Mat(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
    for(ImageMask::const_iterator it = mask.begin(view.getWidth()); it != mask.end(); ++it)
    {
        masked_depth_image.at<float>(*it) = view.getDepth(it->x,it->y);
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

pcl::PointCloud<pcl::PointNormal>::ConstPtr pclToNpcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl, int k_search)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr npcl(new pcl::PointCloud<pcl::PointNormal>);

    if (pcl->points.size() < 3) {
        return npcl;
    }

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(k_search);

    norm_est.setInputCloud(pcl);

    norm_est.compute(*npcl);

    // Copy the points to the point_normals cloud
    pcl::copyPointCloud(*pcl, *npcl);

    return npcl;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

pcl::PointCloud<pcl::PointXYZ>::ConstPtr downSamplePcl(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl, double leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (pcl);
    sor.setLeafSize (leaf_size,leaf_size,leaf_size);
    sor.filter (*output);

    return output;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void findEuclideanClusters(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const PointCloudMaskPtr& mask,
                           double tolerance, int min_cluster_size, std::vector<PointCloudMaskPtr>& clusters)
{
    if (cloud->size() == 0) {
        return;
    }
    if (mask->size() == 0) {
        return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud, mask);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.setIndices (mask);

    std::vector<pcl::PointIndices> clusters_pcl;
    ec.extract(clusters_pcl);

    clusters.resize(clusters_pcl.size());
    for(unsigned int i = 0; i < clusters_pcl.size(); ++i)
        clusters[i] = PointCloudMaskPtr(new PointCloudMask(clusters_pcl[i].indices));
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void get2DConvexHull(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pcl, const PointCloudMask& mask, const geo::Pose3D& pose, ConvexHull2D& convex_hull)
{
    assert(!mask.empty());

    // Convex hull calculation using OpenCV
    std::vector<int> chull_mask_indices;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
    if (mask[0] < 0)
    {
        cv::Mat_<cv::Vec2f> pointMat(1, pcl->points.size());

        for (size_t i = 0; i < pcl->size(); i++) {
            geo::Vector3 p = pose * geo::Vector3(pcl->points[i].x, pcl->points[i].y, pcl->points[i].z);
            point_cloud.points.push_back(pcl::PointXYZ(p.x,p.y,p.z));
//            std::cout << "no mask " << p << std::endl;
            pointMat(0, i) = cv::Vec2f(p.x, p.y);
        }

        cv::convexHull(pointMat, chull_mask_indices);
    }
    else
    {
        cv::Mat_<cv::Vec2f> pointMat(1, mask.size());
        for (size_t i = 0; i < mask.size(); i++) {
            geo::Vector3 p = pose * geo::Vector3(pcl->points[mask[i]].x, pcl->points[mask[i]].y, pcl->points[mask[i]].z);
            point_cloud.points.push_back(pcl::PointXYZ(p.x,p.y,p.z));
//            std::cout << "mask " << p << std::endl;
            pointMat(0, i) = cv::Vec2f(p.x, p.y);
        }

        cv::convexHull(pointMat, chull_mask_indices);
    }

    convex_hull.chull.clear();
    for(unsigned int i = 0; i < chull_mask_indices.size(); ++i)
        convex_hull.chull.push_back(point_cloud.points[chull_mask_indices[i]]);

    Eigen::Vector4f min, max; // TODO: This can also be done in the loop
    pcl::getMinMax3D (convex_hull.chull, min, max);
    convex_hull.min_z = min.z(); //convex_hull.min_z = 0.0; //! TODO: This is a hack (min z = 0 ; for debugging)
    convex_hull.max_z = max.z();

    convex_hull.center_point =  geo::Vector3( (min.x()+max.x()) / 2 ,
                                              (min.y()+max.y()) / 2 ,
                                              (min.z()+max.z()) / 2 );
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void add2DConvexHull(const ConvexHull2D& input, ConvexHull2D& output)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);
    *pcl += input.chull;
    *pcl += output.chull;

    get2DConvexHull(pcl, NO_MASK, geo::Pose3D::identity(), output);
}


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

bool polygonCollisionCheck(const ConvexHull2D& chull1, const ConvexHull2D& chull2, double& overlap_factor)
{
    const pcl::PointCloud<pcl::PointXYZ>& p1 = chull1.chull;
    const double& min_z1 = chull1.min_z;
    const double& max_z1 = chull1.max_z;

    const pcl::PointCloud<pcl::PointXYZ>& p2 = chull2.chull;
    const double& min_z2 = chull2.min_z;
    const double& max_z2 = chull2.max_z;

    if (p1.size() < 3 || p2.size() < 3) return false;

    // Check height
    if (max_z2 < min_z1 || max_z1 < min_z2) return false;

    ClipperLib::Path polygon1, polygon2;

    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = p1.begin(); pit != p1.end(); ++pit) {
        polygon1 << ClipperLib::IntPoint(pit->x*1000,pit->y*1000);
    }
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator pit = p2.begin(); pit != p2.end(); ++pit) {
        polygon2 << ClipperLib::IntPoint(pit->x*1000,pit->y*1000);
    }

    if (ClipperLib::Area(polygon1) == 0 || ClipperLib::Area(polygon2) == 0)
        return false;

    double area1 = ClipperLib::Area(polygon1) / (1000*1000);

    ClipperLib::Clipper c;
    c.AddPath(polygon1,ClipperLib::ptSubject, true);
    c.AddPath(polygon2,ClipperLib::ptClip, true);

    ClipperLib::Paths res;
    c.Execute(ClipperLib::ctIntersection,res);

    if (res.size() == 0)
    {
        return false;
    }

    // Calculate the intersection area
    double area = ClipperLib::Area(res[0]) / (1000*1000);

    overlap_factor = area / area1;

    return true;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void removeInViewConvexHullPoints(rgbd::ImageConstPtr rgbd_image, const geo::Pose3D& sensor_pose, ConvexHull2D& convex_hull, float max_range)
{
    for ( pcl::PointCloud<pcl::PointXYZ>::iterator it = convex_hull.chull.begin(); it != convex_hull.chull.end() ; ) {
        geo::Vector3 p(it->x,it->y, (convex_hull.max_z + convex_hull.min_z) / 2);
        bool in_frustrum, object_in_front;
        if ( inView(rgbd_image, sensor_pose, p, max_range, 0.2, in_frustrum, object_in_front) ) { //! TODO: THis param here <--
            it = convex_hull.chull.erase(it);
        } else {
            ++it;
        }
    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

bool inView(rgbd::ImageConstPtr rgbd_image, const geo::Pose3D& sensor_pose, const geo::Vector3& p, float max_range, float padding_fraction, bool& in_frustrum, bool& object_in_front)
{
    in_frustrum = false;
    object_in_front = false;

    geo::Vector3 p_SENSOR = sensor_pose.inverse() * p;
    float p_depth = -p_SENSOR.z;
    if (p_depth < 0)
        return false;

    rgbd::View view(*rgbd_image, 320); //! TODO: This param here <---

    cv::Point2d idx = view.getRasterizer().project3Dto2D(p_SENSOR);

    if (idx.x >= view.getWidth()*padding_fraction &&
            idx.x < view.getWidth() - view.getWidth()*padding_fraction &&
            idx.y >= view.getHeight()*padding_fraction &&
            idx.y < view.getHeight() - view.getHeight()*padding_fraction) {
        in_frustrum = true;
        float img_depth = fabs(view.getDepth(idx.x,idx.y));

        if ( (p_depth < img_depth || img_depth <= 0) && p_depth < max_range ) //! TODO: THIS NEEDS REVISION! -- Already had some but still crucial part!
        {
            return true;
        }
        else
        {
            object_in_front = true;
        }
    }
    return false;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void getDisplacementVector(const ConvexHull2D& c1, const ConvexHull2D& c2, geo::Vector3& dv)
{
    if (c1.chull.size() == 0 || c2.chull.size() == 0)
    {
        dv = geo::Vector3(0,0,0);
        return;
    }

    Eigen::Vector4f input_min, input_max, output_min, output_max;
    pcl::getMinMax3D (c1.chull, input_min, input_max);
    pcl::getMinMax3D (c2.chull, output_min, output_max);

    Eigen::Vector4f d_max = input_max-output_max;
    Eigen::Vector4f d_min = input_min-output_min;

    // Only expansion
    if (d_max[0] < 0) d_max[0] = 0;
    if (d_max[1] < 0) d_max[1] = 0;
    if (d_max[2] < 0) d_max[2] = 0;
    if (d_min[0] > 0) d_min[0] = 0;
    if (d_min[1] > 0) d_min[1] = 0;
    if (d_min[2] > 0) d_min[2] = 0;

    if (-d_min[0] > fabs(d_max[0]))
        dv.x = d_min[0];
    else
        dv.x = d_max[0];

    if (-d_min[1] > d_max[1])
        dv.y = d_min[1];
    else
        dv.y = d_max[1];

    dv.z = 0;
}

}

}

}
