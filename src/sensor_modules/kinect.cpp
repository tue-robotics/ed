#include "ed/sensor_modules/kinect.h"

#include "ed/association_localization_modules/point_normal_alm.h"
#include "ed/association_localization_modules/polygon_height_alm.h"
#include "ed/association_localization_modules/relative_alm.h"

#include "ed/segmentation_modules/euclidean_clustering_sm.h"

#include "ed/measurement.h"
#include "ed/entity.h"
#include "ed/world_model.h"
#include "ed/update_request.h"

#include "ed/helpers/depth_data_processing.h"
#include "ed/helpers/visualization.h"

#include <rgbd/Image.h>

#include <ed/rgbd_data.h>

#include <tue/profiling/scoped_timer.h>

#include <geolib/Shape.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

void createNewEntity(const RGBDData& rgbd_data, const PointCloudMaskPtr& mask, UpdateRequest& req)
{
    // Create the measurement
    MeasurementConstPtr m(new Measurement(rgbd_data, mask));

    req.addMeasurement(ed::Entity::generateID(), m);
}

// ----------------------------------------------------------------------------------------------------

void filterPointsBehindWorldModel(const WorldModelConstPtr& world_model, const geo::Pose3D& sensor_pose, rgbd::ImagePtr rgbd_image, const ros::Publisher& pub)
{
    rgbd::View view(*rgbd_image, 160);

    // Visualize the frustrum
    helpers::visualization::publishRGBDViewFrustrumVisualizationMarker(view, sensor_pose, pub, 0, "frustrum_top_kinect");

    cv::Mat wm_depth_image(view.getHeight(), view.getWidth(), CV_32FC1, 0.0);

    geo::PointerMap pointer_map;    // TODO: GET RID OF THIS
    geo::TriangleMap triangle_map;  // TODO: GET RID OF THIS
    geo::DefaultRenderResult res(wm_depth_image, 0, pointer_map, triangle_map);

    for(WorldModel::const_iterator it = world_model->begin(); it != world_model->end(); ++it)
    {
        const EntityConstPtr& e = *it;

        if (e->shape())
        {
            geo::Pose3D pose = sensor_pose.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            // Render
            view.getRasterizer().render(opt, res);
        }
    }

    const cv::Mat& depth_image = rgbd_image->getDepthImage();
    cv::Mat new_depth_image(depth_image.rows, depth_image.cols, CV_32FC1);

    float f = (float)view.getWidth() / depth_image.cols;

    for(int y = 0; y < depth_image.rows; ++y)
    {
        for(int x = 0; x < depth_image.cols; ++x)
        {
            float d_sensor = depth_image.at<float>(y, x);
            float d_wm = wm_depth_image.at<float>(f * y, f * x);
            if (d_sensor == d_sensor) // Check if point is actually valid
            {
                if ( d_wm == 0 || d_sensor < d_wm )
                {
                    new_depth_image.at<float>(y, x) = depth_image.at<float>(y, x);
                }
                else
                {
                    new_depth_image.at<float>(y, x) = -d_wm; // TODO: <-- This value; Valid point @ world model object ( used for not taking int account when rendering but taking into account @ clearing )
                }
            }
            else
            {
                new_depth_image.at<float>(y, x) = -d_wm;
            }
        }
    }

    rgbd_image->setDepthImage(new_depth_image);
}

// ----------------------------------------------------------------------------------------------------

Kinect::Kinect(const tf::TransformListener& tf_listener) : SensorModule(tf_listener, "kinect")
{
}

void Kinect::configure(tue::Configuration config, bool reconfigure)
{
    if (config.readArray("association_modules"))
    {
        while(config.nextArrayItem())
        {
            std::string type;
            if (config.value("type", type))
            {
                if (type == "PointNormal")
                {
                    std::cout << "Loading RGBD Association module type '" << type << "'" << std::endl;
                    RGBDALModulePtr alm(new PointNormalALM());
                    alm->configure(config);
                    al_modules_[type] = alm;
                }
                else if (type == "PolygonHeight")
                {
                    std::cout << "Loading RGBD Association module type '" << type << "'" << std::endl;
                    RGBDALModulePtr alm(new PolygonHeightALM());
                    alm->configure(config);
                    al_modules_[type] = alm;
                }
                else if (type == "RelativeLocalization")
                {
                    std::cout << "Loading RGBD Association module type '" << type << "'" << std::endl;
                    RGBDALModulePtr alm(new RelativeLocalizationModule());
                    alm->configure(config);
                    al_modules_[type] = alm;
                }
                else
                {
                    std::cout << "Unknown RGBD Association module type '" << type << "'" << std::endl;
                }
            }
        }

        config.endArray();
    }

    if (config.readArray("segmentation_modules"))
    {
        while(config.nextArrayItem())
        {
            std::string type;
            if (config.value("type", type))
            {
                if (type == "EuclideanClustering")
                {
                    RGBDSegModulePtr seg(new EuclideanClusteringSM());
                    seg->configure(config);
                    seg_modules_[type] = seg;
                }
                else
                {
                    std::cout << "Unknown RGBD Segmentation module type '" << type << "'" << std::endl;
                }
            }
        }

        config.endArray();
    }

    // get sensor config
    std::string source_str, frame_str;
    if (config.value("source", source_str) && config.value("frame_id", frame_str))
    {
        source_ = source_str;
        frame_ = frame_str;

        // Initialize profiler
        profiler_.setName("kinect_sensor");
        pub_profile_.initialize(profiler_);

        // Initialize kinect sensor
        rgbd_client_.intialize(source_str);
    }

    // Get tunable params
    config.value("voxel_size", voxel_size_);
    config.value("max_range", max_range_);
    config.value("clearing_padding_fraction", clearing_padding_fraction_);
    config.value("normal_k_search", normal_k_search_);
    config.value("visualize", visualize_);
}

void Kinect::update(const WorldModelConstPtr& world_model, UpdateRequest& req)
{

    tue::ScopedTimer timer_total(profiler_, "total");

    //! 0) Receive new image
    rgbd::ImagePtr rgbd_image;
    {
        tue::ScopedTimer t(profiler_, "0) Retrieve image");
        rgbd_image = rgbd_client_.nextImage();
        if (!rgbd_image)
        {
            ROS_WARN_STREAM("No RGBD image available for sensor '" << source_ << "', is the RGBD Server running?");
            return;
        }
    }

    //! 1) Lookup sensor map transform
    geo::Pose3D sensor_pose, sensor_pose_improved(0.0,0.0,0.0);
    {
        tue::ScopedTimer t(profiler_, "1) lookup sensor_tf");

//        if(sensor_pose_improved)
        if(!getSensorPoseMap(rgbd_image->getTimestamp(), sensor_pose))
        {
            ROS_WARN_STREAM("No sensor pose available for sensor '" << source_ << "'");
            return;
        }
        else
        {
            geo::Pose3D corr;
            corr.setOrigin(geo::Vector3(0, 0, 0));
            corr.setRPY(3.1415, 0, 0);
            sensor_pose = sensor_pose* corr; // geolib fix
        }
    }

    //! 2) Preprocess image: remove all data points that are behind world model entities (Why?)
    {
        tue::ScopedTimer t(profiler_, "2) filter points behind wm");
        filterPointsBehindWorldModel(world_model, sensor_pose, rgbd_image, vis_marker_pub_);
    }

    // Construct RGBD Data
    RGBDData rgbd_data;
    rgbd_data.image = rgbd_image;
    rgbd_data.sensor_pose = sensor_pose;

    //! 3) Convert rgbd data to voxelized point cloud
    {
        tue::ScopedTimer t(profiler_, "3) create sensor point cloud");

        helpers::ddp::extractPointCloud(rgbd_data, voxel_size_, max_range_, 1);
    }

    //! 4) Calculate point cloud normals
    {
        tue::ScopedTimer t(profiler_, "4) calculate sensor pc normals");
        helpers::ddp::calculatePointCloudNormals(rgbd_data, normal_k_search_);
    }

    if (visualize_)
        helpers::visualization::publishNpclVisualizationMarker(rgbd_data.sensor_pose, rgbd_data.point_cloud_with_normals,
                                                               vis_marker_pub_, 0, "sensor_npcl");

    //! 5) Create point cloud mask: put all point cloud indices in the mask
    PointCloudMaskPtr pc_mask(new PointCloudMask(rgbd_data.point_cloud->points.size()));
    for(unsigned int i = 0; i < pc_mask->size(); ++i)
        (*pc_mask)[i] = i;


    //! 6) Do the Association and Localization on the entities in the frustrum (UPDATING)
    {
        tue::ScopedTimer t(profiler_, "6) association and localisation");

        ALMResult alm_result;

        for (std::map<std::string, RGBDALModulePtr>::const_iterator it = al_modules_.begin(); it != al_modules_.end(); ++it) {

            if (pc_mask->empty())
                break;
            profiler_.startTimer(it->second->getType());
            it->second->process(rgbd_data, pc_mask, world_model, alm_result);
            profiler_.stopTimer();
        }

        // Process ALM result
        for(std::map<UUID, std::vector<MeasurementConstPtr> >::const_iterator it = alm_result.associations.begin();
            it != alm_result.associations.end(); ++it)
        {
            const std::vector<MeasurementConstPtr>& measurements = it->second;
            req.addMeasurements(it->first, measurements);
        }
    }


    //! 7) Segment the residual sensor data
    if (!pc_mask->empty())
    {
        tue::ScopedTimer t(profiler_, "7) Segmentation");

        std::vector<PointCloudMaskPtr> segments;
        segments.push_back(pc_mask);

        profiler_.startTimer("segmentation");
        for (std::map<std::string, RGBDSegModulePtr>::const_iterator it = seg_modules_.begin(); it != seg_modules_.end(); ++it) {
            it->second->process(rgbd_data, segments);
        }
        profiler_.stopTimer();

        profiler_.startTimer("add new entities");

        //! 8) Add the segments as new entities
        for (std::vector<PointCloudMaskPtr>::const_iterator it = segments.begin(); it != segments.end(); ++it)
        {
            createNewEntity(rgbd_data, *it, req);
        }

        profiler_.stopTimer();
    }


    //! 8) Clearing
    {
        tue::ScopedTimer t(profiler_, "8) clearing");

        std::vector<UUID> entities_in_view_not_associated;
        for (WorldModel::const_iterator it = world_model->begin(); it != world_model->end(); ++it)
        {
            const EntityConstPtr& e = *it;

            if (!e->shape()) //! if it has no shape
            {
                bool in_frustrum, object_in_front;
                if (helpers::ddp::inView(rgbd_data.image, rgbd_data.sensor_pose, e->convexHull().center_point, max_range_, clearing_padding_fraction_, in_frustrum, object_in_front))
                {
                    MeasurementConstPtr m = e->lastMeasurement();

                    if (m && ros::Time::now().toSec() - m->timestamp() > 1.0)
                    {
                        entities_in_view_not_associated.push_back(e->id());
                    }
                }
            }
        }

        for (std::vector<UUID>::const_iterator it = entities_in_view_not_associated.begin(); it != entities_in_view_not_associated.end(); ++it)
            req.removeEntity(*it);
    }


    //! 8) Visualization
    if (visualize_)
    {
        WorldModel wm_new(*world_model);
        wm_new.update(req);
        helpers::visualization::showMeasurements(wm_new, rgbd_data.image);
    }

    pub_profile_.publish();
}

}
