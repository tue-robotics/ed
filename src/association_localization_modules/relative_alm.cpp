#include "ed/association_localization_modules/relative_alm.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>
#include <opencv/highgui.h>
#include <pcl/registration/icp.h>

#include "ed/helpers/depth_data_processing.h"
#include "ed/helpers/visualization.h"
#include "ed/association_localization_modules/world_model_renderer.h"
#include "ed/entity.h"

namespace ed
{

RelativeLocalizationModule::RelativeLocalizationModule() : RGBDALModule("relative_localization")
{
}

void RelativeLocalizationModule::configure(tue::Configuration config)
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

        std::cout << "Parameters relative localization association module: \n" <<
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

void RelativeLocalizationModule::process(const RGBDData& sensor_data,
                                         PointCloudMaskPtr& not_associated_mask,
                                         const WorldModelConstPtr& world_model,
                                         ALMResult& result)
{

//    broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "camera_link"));

    //! 1) Get the world model point cloud
    pcl::PointCloud<pcl::PointNormal>::ConstPtr world_model_npcl;
    std::vector<const Entity*> world_model_pc_entity_ptrs;

    // Create a render view
    rgbd::View sensor_view(*sensor_data.image, render_width_);

    // Render the view
    WorldModelRenderer wmr;
    cv::Mat wm_depth_img = cv::Mat::zeros(sensor_view.getHeight(), sensor_view.getWidth(), CV_32F);
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_model_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    wmr.render(sensor_data.sensor_pose, world_model, render_max_range_, sensor_view, wm_depth_img, *world_model_pcl, world_model_pc_entity_ptrs);
    wmr.~WorldModelRenderer();

    // Downsample the pointcloud (why?)
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr world_model_pcl_downsampled = helpers::ddp::downSamplePcl(world_model_pcl, render_voxel_size_);
    world_model_pcl->~PointCloud();

    // Calculate the normals with use of pcl (why?)
    world_model_npcl =  helpers::ddp::pclToNpcl(world_model_pcl_downsampled, normal_k_search_);

    //    if (visualize_)
    //        helpers::visualization::publishNpclVisualizationMarker(sensor_data.sensor_pose, world_model_npcl, vis_marker_pub_, 1, "world_model_npcl");


    //! 2) Perform association
    // Create vector of pointers to entities with which the data points are associated.
    std::vector<EntityConstPtr> associations(sensor_data.point_cloud_with_normals->size());
    //    std::vector<const ed::Entity*> observed_entities;
    //    observed_entities.clear();
    std::map< EntityConstPtr, pcl::PointCloud<pcl::PointNormal> > sensor_association_map, wm_association_map;

    //    associate(rgbd_data.point_cloud_with_normals, world_model_npcl, world_model_pc_entity_ptrs, associations);

    // If something went wrong, return
    if (world_model_npcl->size() == 0 || !tree_)
        return;

    // Set point representation of KdTree and set input point cloud (world model point cloud with normals)
    tree_->setPointRepresentation (boost::make_shared<const AssociationPRRelativeALM> (point_representation_));
    tree_->setInputCloud(world_model_npcl);

    // Loop over (empty) vector of associated points
    for (unsigned int i = 0; i < associations.size(); ++i)
    {
        // Declaration of vector to contain index to a point in the world model that is associated with point i in the sensor data
        std::vector<int> k_indices(1);
        // Declaration of vector to contain the squared distance between those points
        std::vector<float> k_sqr_distances(1);

        // Perform the radius search in the world model point cloud from point i in the sensor data point cloud
        if (tree_->radiusSearch(*sensor_data.point_cloud_with_normals, i, association_correspondence_distance_, k_indices, k_sqr_distances, 1))
        {
            // Store pointer to associated entity in associations
            EntityConstPtr ecp(world_model_pc_entity_ptrs[k_indices[0]]);
            associations[i] = ecp;

            sensor_association_map[ecp] += pcl::PointCloud<pcl::PointNormal>(*sensor_data.point_cloud_with_normals,std::vector<int>(1,i));
            wm_association_map[ecp] += pcl::PointCloud<pcl::PointNormal>(*world_model_npcl,k_indices);
        }
        else {
            // If sensor data point is not associated...
            EntityConstPtr ecp;
            associations[i] = ecp;
        }
    }

    //! 3) Match scan data with their respective entities in the world model and get transforms from that
    for (std::map<EntityConstPtr, pcl::PointCloud<pcl::PointNormal> >::iterator it = sensor_association_map.begin(); it != sensor_association_map.end(); it++) {
        EntityConstPtr entity = it->first;
        pcl::PointCloud<pcl::PointNormal>::ConstPtr sensor_entity_pc(&(it->second));
        pcl::PointCloud<pcl::PointNormal>::ConstPtr world_model_entity_pc(&(wm_association_map[entity]));

        pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setInputSource(sensor_entity_pc);
        icp.setInputTarget(world_model_entity_pc);
        pcl::PointCloud<pcl::PointNormal> final_pc;
        icp.align(final_pc);
        icp.getFinalTransformation();
    }


    //! 4) Update the mask
    not_associated_mask->clear();

    for(unsigned int i = 0; i < associations.size(); ++i)
        if (!associations[i])
            not_associated_mask->push_back(i);

    if (visualize_)
    {
        pcl::PointCloud<pcl::PointNormal>::Ptr residual_npcl(new pcl::PointCloud<pcl::PointNormal>);
        for(unsigned int i = 0; i < associations.size(); ++i)
            if (!associations[i])
                residual_npcl->push_back(sensor_data.point_cloud_with_normals->points[i]);

        helpers::visualization::publishNpclVisualizationMarker(sensor_data.sensor_pose, residual_npcl, vis_marker_pub_, 2, "residual_npcl");
    }

    pub_profile_.publish();
}

} // namespace ed
