#include "ed/association_localization_modules/world_model_renderer.h"
#include "ed/entity.h"

#include <geolib/Shape.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

class SampleRenderResult : public geo::RenderResult {

public:

    SampleRenderResult(cv::Mat& z_buffer, int width, int height, const Entity* e, const geo::DepthCamera& rasterizer, pcl::PointCloud<pcl::PointXYZ>& pc,
                        std::vector<const Entity*>& entities, std::vector<int>& triangle_indices, float max_range)
                : geo::RenderResult(width, height), z_buffer_(z_buffer), entity_(e), rasterizer_(rasterizer), pc_(pc), entities_(entities), triangles_indices_(triangle_indices), max_range_(max_range)

    {
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer_.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer_.at<float>(y, x) = depth;

            if (depth < max_range_)
            {

                pcl::PointXYZ p;
                p.x = rasterizer_.project2Dto3DX(x) * depth;
                p.y = rasterizer_.project2Dto3DY(y) * depth;
                p.z = -depth;

                pc_.push_back(p);
                entities_.push_back(entity_);
                triangles_indices_.push_back(i_triangle);
            }
        }
    }

protected:

    cv::Mat z_buffer_;
    const Entity* entity_;
    const geo::DepthCamera& rasterizer_;
    pcl::PointCloud<pcl::PointXYZ>& pc_;
    std::vector<const Entity*>& entities_;
    std::vector<int>& triangles_indices_;
    float max_range_;

};

// ----------------------------------------------------------------------------------------------------

WorldModelRenderer::WorldModelRenderer()
{
}

// ----------------------------------------------------------------------------------------------------

WorldModelRenderer::~WorldModelRenderer()
{
}

// ----------------------------------------------------------------------------------------------------

void WorldModelRenderer::render(const geo::Pose3D& camera_pose, const std::map<UUID, EntityConstPtr>& entities, float max_range, const rgbd::View& view, cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>& pc, std::vector<const Entity*>& pc_entity_ptrs)
{
    for(std::map<UUID, EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const EntityConstPtr& e = it->second;
        if (e->shape())
        {
            geo::Pose3D pose = camera_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            std::vector<int> triangle_indices;
            SampleRenderResult res(img, view.getWidth(), view.getHeight(), e.get(), view.getRasterizer(), pc, pc_entity_ptrs, triangle_indices, max_range);

            // Render
            view.getRasterizer().render(opt, res);
        }
    }
}

}
