#include "ed/association_localization_modules/relative_alm.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>
#include <opencv/highgui.h>
#include <pcl/registration/icp.h>

#include "ed/helpers/depth_data_processing.h"
#include "ed/helpers/visualization.h"
#include "ed/association_localization_modules/world_model_renderer.h"
#include "ed/entity.h"

#include <tue/profiling/scoped_timer.h>

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

geo::Transform RelativeLocalizationModule::eigenMat2geoTransform(Eigen::Matrix<float,4,4> T) {
    geo::Transform Transformation;

    Transformation.R.xx = T(0,0);
    Transformation.R.xy = T(0,1);
    Transformation.R.xz = T(0,2);

    Transformation.R.yx = T(1,0);
    Transformation.R.yy = T(1,1);
    Transformation.R.yz = T(1,2);

    Transformation.R.zx = T(2,0);
    Transformation.R.zy = T(2,1);
    Transformation.R.zz = T(2,2);

    Transformation.t.x = T(0,3);
    Transformation.t.y = T(1,3);
    Transformation.t.z = T(2,3);
    return Transformation;
}

void RelativeLocalizationModule::process(RGBDData& sensor_data,
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
        wmr.render(sensor_data.sensor_pose, world_model, render_max_range_, sensor_view, wm_depth_img, *world_model_pcl, world_model_pc_entity_ptrs);

        profiler_.stopTimer();

        // Downsample the pointcloud (why?)
        profiler_.startTimer("downsample");
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr world_model_pcl_downsampled = helpers::ddp::downSamplePcl(world_model_pcl, render_voxel_size_);
        profiler_.stopTimer();

        // Calculate the normals with use of pcl
        profiler_.startTimer("calculate normals");
        world_model_npcl =  helpers::ddp::pclToNpcl(world_model_pcl_downsampled, normal_k_search_);
        profiler_.stopTimer();
    }

    if (visualize_)
        helpers::visualization::publishNpclVisualizationMarker(sensor_data.sensor_pose, world_model_npcl, vis_marker_pub_, 1, "world_model_npcl");

    //! 3.1) Match all depth data with world model render to update sensor pose
    pcl::PointCloud<pcl::PointNormal> final_pc;
    {
        pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setInputSource(world_model_npcl);
        icp.setInputTarget(sensor_data.point_cloud_with_normals);
        icp.align(final_pc);

        Eigen::Matrix<float, 4, 4> T = icp.getFinalTransformation();

        sensor_data.sensor_pose = sensor_data.sensor_pose * RelativeLocalizationModule::eigenMat2geoTransform(T);
    }

    //! 2) Perform point normal association
    // Create vector of pointers to entities with which the data points are associated.
    std::vector<const ed::Entity*> associations(sensor_data.point_cloud_with_normals->size());
    std::map< const ed::Entity*, pcl::PointCloud<pcl::PointNormal>::Ptr > sensor_association_map, wm_association_map;

    {
        tue::ScopedTimer t(profiler_, "2) association");

        // If something went wrong, return
        if (world_model_npcl->size() == 0 || !tree_)
            return;

        {
            tue::ScopedTimer setinputcloud_timer(profiler_, "setinputcloud");
            // Set point representation of KdTree and set input point cloud (world model point cloud with normals)
            tree_->setPointRepresentation (boost::make_shared<const AssociationPRRelativeALM> (point_representation_));
            tree_->setInputCloud(world_model_npcl);
        }

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
                associations[i] = world_model_pc_entity_ptrs[k_indices[0]];
                if (!sensor_association_map[associations[i]] && !wm_association_map[associations[i]])
                {
                    // Create new point clouds to save associated entity points in
                    pcl::PointCloud<pcl::PointNormal>::Ptr ptr_to_empty_pc1(new pcl::PointCloud<pcl::PointNormal>);
                    pcl::PointCloud<pcl::PointNormal>::Ptr ptr_to_empty_pc2(new pcl::PointCloud<pcl::PointNormal>);

                    // Add these empty point clouds to a (new) map entry with entity pointer as key
                    sensor_association_map[associations[i]] = ptr_to_empty_pc1;
                    wm_association_map[associations[i]] = ptr_to_empty_pc2;
                }
                *sensor_association_map[associations[i]] += pcl::PointCloud<pcl::PointNormal>(*sensor_data.point_cloud_with_normals,std::vector<int>(1,i));
                *wm_association_map[associations[i]] += pcl::PointCloud<pcl::PointNormal>(*world_model_npcl,k_indices);
            }
            else {
                // If sensor data point is not associated...
                associations[i] = 0;
            }
        }
    }

    //! 3) Match scan data with their respective entities in the world model and get transforms from that


    //! 3.2) Match separte entity point cloud data with associated world model points to update entity poses
//    for (std::map<const ed::Entity*, pcl::PointCloud<pcl::PointNormal>::Ptr >::iterator it = sensor_association_map.begin(); it != sensor_association_map.end(); it++) {
//        const ed::Entity* entity = it->first;

//        pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
//        icp.setInputSource(it->second);
//        icp.setInputTarget(wm_association_map[entity]);
//        pcl::PointCloud<pcl::PointNormal> final_pc;
//        icp.align(final_pc);

//        Eigen::Matrix<float, 4, 4> T = icp.getFinalTransformation();
//        std::cout << "Transformation matrix T = \n" << T << std::endl;
//    }

    //! 4) Update the mask
    {
        tue::ScopedTimer t(profiler_, "mask_update");

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
    }

    pub_profile_.publish();
} // process method

} // namespace ed
