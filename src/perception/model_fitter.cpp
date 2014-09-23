#include "ed/perception/model_fitter.h"

#include <ed/models/models.h>
#include <ed/measurement.h>
#include <ed/entity.h>

#include <rgbd/View.h>

#include <geolib/Shape.h>
#include <geolib/Mesh.h>
#include <geolib/sensors/DepthCamera.h>

#include <opencv2/highgui/highgui.hpp>

namespace ed
{

namespace model_fitter
{

// ----------------------------------------------------------------------------------------------------

class ModelFitterRenderResult : public geo::RenderResult {

public:

    ModelFitterRenderResult(cv::Mat& z_buffer_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_), p_min(1e8, 1e8), p_max(0, 0),
          total_depth(0), min_depth(1e10), n_pixels(0)
    {
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0)
        {
            z_buffer.at<float>(y, x) = depth;
            total_depth += depth;
            ++n_pixels;
        }
        else if (old_depth > depth)
        {
            z_buffer.at<float>(y, x) = depth;
            total_depth = total_depth - old_depth + depth;
        }

        p_min.x = std::min(p_min.x, x);
        p_min.y = std::min(p_min.y, y);
        p_max.x = std::max(p_max.x, x);
        p_max.y = std::max(p_max.y, y);
        min_depth = std::min(min_depth, depth);
    }

    cv::Mat& z_buffer;
    cv::Point2i p_min, p_max;
    double total_depth;
    float min_depth;
    int n_pixels;

};

//// ----------------------------------------------------------------------------------------------------

//ModelFitter::ModelFitter() : ed::PerceptionModule("model_fitter")
//{
//}

//// ----------------------------------------------------------------------------------------------------

//ModelFitter::~ModelFitter()
//{
//}

// ----------------------------------------------------------------------------------------------------

//void ModelFitter::loadModel(const std::string& model_name, const std::string& model_path)
//{
//    ed::models::Loader loader;
//    geo::ShapePtr shape = loader.loadShape(model_name);

//    if (shape)
//    {
//        // Normalize the position of the shape: the origin must be
//        // - (x, y): center of axis aligned bounding box (AABB)
//        // -  z    : the lowest z-value must be 0

//        const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();

//        geo::Vector3 p_min = vertices[0];
//        geo::Vector3 p_max = vertices[0];
//        double z_min = vertices[0].z;

//        for(unsigned int i = 1; i < vertices.size(); ++i)
//        {
//            const geo::Vector3& p = vertices[i];
//            p_min.x = std::min(p_min.x, p.x);
//            p_min.y = std::min(p_min.y, p.y);
//            p_max.x = std::max(p_max.x, p.x);
//            p_max.y = std::max(p_max.y, p.y);
//            z_min = std::min(z_min, p.z);
//        }

//        geo::Pose3D T;
//        T.t = -(p_min + p_max) / 2;
//        T.t.z = -z_min;
//        T.R = geo::Matrix3::identity();

//        geo::Mesh mesh_transformed = shape->getMesh().getTransformed(T);
//        shape->setMesh(mesh_transformed);

//        models_[model_name] = shape;
//    }
//}

// ----------------------------------------------------------------------------------------------------

//ed::PerceptionResult ModelFitter::process(const ed::Measurement& msr) const
//{
//    ed::PerceptionResult res;

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    const pcl::PointCloud<pcl::PointXYZ>& convex_hull_points = msr.convexHull().chull;

//    geo::Vector3 msr_points_total(0, 0, 0);
//    for(unsigned int i = 0; i < convex_hull_points.points.size(); ++i)
//    {
//        const pcl::PointXYZ& p = convex_hull_points.points[i];
//        msr_points_total += geo::Vector3(p.x, p.y, 0);
//    }

//    geo::Vector3 msr_points_center = msr_points_total / convex_hull_points.points.size();

//    geo::Pose3D pose_MAP;
//    pose_MAP.t = msr_points_center;
//    pose_MAP.R = geo::Matrix3::identity();

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    rgbd::View view(*msr.image(), sample_width_);
//    const geo::DepthCamera& rasterizer = view.getRasterizer();

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    const ed::Mask& mask = msr.mask();

//    float view_mask_factor = (float)view.getWidth() / mask.width();

//    // Get min and max of measurement mask
//    cv::Point2i p_mask_min = mask.points[0];
//    cv::Point2i p_mask_max = mask.points[0];

//    for(unsigned int i = 0; i < mask.points.size(); ++i)
//    {
//        const cv::Point2i& p = mask.points[i];

//        p_mask_min.x = std::min(p_mask_min.x, p.x);
//        p_mask_min.y = std::min(p_mask_min.y, p.y);
//        p_mask_max.x = std::max(p_mask_max.x, p.x);
//        p_mask_max.y = std::max(p_mask_max.y, p.y);
//    }

//    // convert to view resolution
//    p_mask_min = view_mask_factor * p_mask_min;
//    p_mask_max = view_mask_factor * p_mask_max;

//    int pixel_width_mask = p_mask_max.x - p_mask_min.x;

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    for(std::map<std::string, geo::ShapeConstPtr>::const_iterator it = models_.begin(); it != models_.end(); ++it)
//    {
//        const std::string& model_name = it->first;
//        const geo::ShapeConstPtr& shape = it->second;

//        float min_width_factor = 0.1;
//        geo::Pose3D best_pose_MAP = pose_MAP;

//        for(float alpha = 0; alpha < 6.28; alpha += 0.2)
//        {
//            pose_MAP.R.setRPY(0, 0, alpha);

//            geo::Pose3D pose_SENSOR = msr.sensorPose().inverse() * pose_MAP;

//            geo::RenderOptions opt;
//            opt.setMesh(shape->getMesh(), pose_SENSOR);

//            cv::Mat model_depth_image(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
//            ModelFitterRenderResult render_res(model_depth_image);

//            rasterizer.render(opt, render_res);

//            int pixel_width_render = render_res.p_max.x - render_res.p_min.x;

//            int pixel_width_diff = std::abs(pixel_width_mask - pixel_width_render);

//            float width_factor = (float)pixel_width_diff / pixel_width_render;

////            std::cout << "pose_SENSOR = " << pose_SENSOR << " width_factor = " << width_factor << std::endl;

//            if (width_factor < min_width_factor)
//            {
//                std::cout << "Measurement: " << p_mask_min << " - " << p_mask_max << std::endl;
//                std::cout << "Render:      " << render_res.p_min << " - " << render_res.p_max
//                          << " min depth = " << render_res.min_depth << std::endl;

//                res.setVisualization("depth_image (" + model_name + ")", render_res.z_buffer / 8);

//                best_pose_MAP = pose_MAP;
//                min_width_factor = width_factor;
//            }
//        }

//        if (min_width_factor < 0.1)
//            std::cout << "Pose (MAP): " << best_pose_MAP << std::endl;

//    }

//    return res;
//}

bool fit(const Entity& e, const geo::ShapeConstPtr& shape, geo::Pose3D& pose)
{
    MeasurementConstPtr m = e.bestMeasurement();
    if (!m)
        return false;

    if (e.convexHull().chull.empty())
        return false;

    geo::Pose3D expected_pose_MAP(geo::Matrix3::identity(), e.convexHull().center_point);
    expected_pose_MAP.t.z = 0;

    int sample_width = 64;

    rgbd::View view(*m->image(), sample_width);

    int n_a_samples = 4;

    float best_score = 0;
    geo::Pose3D best_pose_MAP;

    float cutoff_distance = 0.2;

    for(double dx = -0.3; dx < 0.31; dx += 0.1)
    {
        for(double dy = -0.3; dy < 0.3; dy += 0.1)
        {
            for(double a = 0; a < 6.283; a += 6.283 / n_a_samples)
            {
                geo::Pose3D test_pose_MAP;
                test_pose_MAP.t = expected_pose_MAP.t + geo::Vector3(dx, dy, 0);
                test_pose_MAP.setRPY(0, 0, a);

                geo::Pose3D test_pose_SENSOR = m->sensorPose().inverse() * test_pose_MAP;

                geo::RenderOptions opt;
                opt.setMesh(shape->getMesh(), test_pose_SENSOR);

                cv::Mat model_depth_image(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);
                ModelFitterRenderResult render_res(model_depth_image);

                view.getRasterizer().render(opt, render_res);

//                cv::imshow("model", model_depth_image / 8);
//                cv::waitKey();

                float max_possible_score = 0;
                float score = 0;

                for(int ix = 0; ix < view.getWidth(); ++ix)
                {
                    for(int iy = 0; iy < view.getHeight(); ++iy)
                    {
                        float d_sensor = view.getDepth(ix, iy);
                        float d_model = model_depth_image.at<float>(ix, iy);
                        if (d_sensor > 0 && d_sensor == d_sensor && d_model > 0)
                        {
                            float diff = d_sensor - d_model;
                            if (std::abs(diff) < cutoff_distance)
                            {
                                float s = cutoff_distance - diff;
                                score += s;
                            }

                            max_possible_score += cutoff_distance * cutoff_distance;
                        }
                    }
                }

//                float score = (float)num_explained_pixels / num_total_pixels;

                float normalized_score = score / max_possible_score;

                if (max_possible_score > 0 && normalized_score > best_score)
                {
                    best_pose_MAP = test_pose_MAP;
                    best_score = normalized_score;
                }
            }  // a
        } // dy
    } // dx

    std::cout << "best_score: " << best_score << std::endl;

    if (best_score > 0.1)
    {
        pose = best_pose_MAP;
        return true;
    }

    return false;
}

}

}
