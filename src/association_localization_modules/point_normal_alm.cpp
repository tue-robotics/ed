#include "ed/association_localization_modules/point_normal_alm.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>
#include <opencv/highgui.h>

#include "ed/helpers/depth_data_processing.h"
#include "ed/helpers/visualization.h"
#include "ed/association_localization_modules/world_model_renderer.h"
#include "ed/entity.h"
#include "ed/time.h"

#include <tue/profiling/scoped_timer.h>

namespace ed
{

PointNormalALM::PointNormalALM() : RGBDALModule("point_normal")
{
}

void PointNormalALM::configure(tue::Configuration config)
{
    if (config.readGroup("parameters"))
    {
        config.value("association_correspondence_distance", association_correspondence_distance_);
        config.value("position_weight", position_weight_);
        config.value("normal_weight", normal_weight_);
        config.value("visualize", visualize_);
        config.value("render_width", render_width_);
        config.value("render_max_range", render_max_range_);
        config.value("render_voxel_size", render_voxel_size_);
        config.value("normal_k_search", normal_k_search_);

        std::cout << "Parameters point normal association: \n" <<
            "- association_correspondence_distance: " << association_correspondence_distance_ << "\n" <<
            "- position_weight: " << position_weight_ << "\n" <<
            "- normal_weight: " << normal_weight_ << "\n" <<
            "- render_width: " << render_width_ << "\n" <<
            "- render_max_range: " << render_max_range_ << "\n" <<
            "- render_voxel_size_: " << render_voxel_size_ << "\n" <<
            "- normal_k_search: " << normal_k_search_ << "\n" <<
            "- visualize: " << visualize_ << std::endl;

        config.endGroup();
    }

    float pw = position_weight_;
    float cw = normal_weight_;
    float alpha[6] = {pw,pw,pw,cw,cw,cw};
    point_representation_.setRescaleValues (alpha);

    tree_ = pcl::KdTreeFLANN<pcl::PointNormal>::Ptr(new pcl::KdTreeFLANN<pcl::PointNormal>);
}

void PointNormalALM::process(const RGBDData& sensor_data,
                             PointCloudMaskPtr& not_associated_mask,
                             const WorldModelConstPtr& world_model,
                             ALMResult& result)
{

    //! 1) Get the world model point cloud
    pcl::PointCloud<pcl::PointNormal>::ConstPtr world_model_npcl;
    std::vector<const Entity*> world_model_pc_entity_ptrs;
    {
        tue::ScopedTimer t(profiler_, "1) get_world_model_pcl");

        // Create a render view
        rgbd::View sensor_view(*sensor_data.image, render_width_);

        profiler_.startTimer("render");

        // Render the view
        WorldModelRenderer wmr;
        cv::Mat wm_depth_img = cv::Mat::zeros(sensor_view.getHeight(), sensor_view.getWidth(), CV_32F);
        pcl::PointCloud<pcl::PointXYZ>::Ptr world_model_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        wmr.render(sensor_data.image->getFrameId(), sensor_data.image->getTimestamp(), world_model, render_max_range_, sensor_view, wm_depth_img, *world_model_pcl, world_model_pc_entity_ptrs);

        profiler_.stopTimer();

        //! Downsample the pointcloud
        profiler_.startTimer("down sample");
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr world_model_pcl_downsampled = helpers::ddp::downSamplePcl(world_model_pcl, render_voxel_size_);
        profiler_.stopTimer();

        //! Calculate the normals with use of pcl
        profiler_.startTimer("calculate normals");
        world_model_npcl =  helpers::ddp::pclToNpcl(world_model_pcl_downsampled, normal_k_search_);
        profiler_.stopTimer();
    }

    // TODO rel-poses : visualize Normal point clouds
//    if (visualize_)
//        helpers::visualization::publishNpclVisualizationMarker(sensor_data.sensor_pose, world_model_npcl, vis_marker_pub_, 1, "world_model_npcl");

    //! 2) Perform the point normal association
    std::vector<const ed::Entity*> associations(sensor_data.point_cloud_with_normals->size());
    {
        tue::ScopedTimer t(profiler_, "2) association");

        //    associate(rgbd_data.point_cloud_with_normals, world_model_npcl, world_model_pc_entity_ptrs, associations);


        if (world_model_npcl->size() == 0 || !tree_)
            return;

        {
            tue::ScopedTimer setinputcloud_timer(profiler_, "setinputcloud");
            tree_->setPointRepresentation (boost::make_shared<const AssociationPR> (point_representation_));
            tree_->setInputCloud(world_model_npcl);
        }

        for (unsigned int i = 0; i < associations.size(); ++i)
        {

            // Find corresponding indices and sqr distances
            std::vector<int> k_indices(1);
            std::vector<float> k_sqr_distances(1);

            // Set pointers
            if (tree_->radiusSearch(*sensor_data.point_cloud_with_normals, i, association_correspondence_distance_, k_indices, k_sqr_distances, 1))
                associations[i] = world_model_pc_entity_ptrs[k_indices[0]];
            else
                associations[i] = 0;
        }
    }

    //! 4) Update the mask
    {
        tue::ScopedTimer t(profiler_, "mask_update");

        not_associated_mask->clear();

        for(unsigned int i = 0; i < associations.size(); ++i)
            if (!associations[i])
                not_associated_mask->push_back(i);

        // TODO rel-poses : visualize
//        if (visualize_)
//        {
//            pcl::PointCloud<pcl::PointNormal>::Ptr residual_npcl(new pcl::PointCloud<pcl::PointNormal>);
//            for(unsigned int i = 0; i < associations.size(); ++i)
//                if (!associations[i])
//                    residual_npcl->push_back(sensor_data.point_cloud_with_normals->points[i]);

//            helpers::visualization::publishNpclVisualizationMarker(sensor_data.sensor_pose, residual_npcl, vis_marker_pub_, 2, "residual_npcl");
//        }
    }

    pub_profile_.publish();
}

}
