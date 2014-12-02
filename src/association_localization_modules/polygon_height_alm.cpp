#include "ed/association_localization_modules/polygon_height_alm.h"

#include "ed/helpers/visualization.h"
#include "ed/helpers/depth_data_processing.h"

#include "ed/entity.h"
#include "ed/world_model.h"
#include "ed/measurement.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>

namespace ed
{

PolygonHeightALM::PolygonHeightALM() : RGBDALModule("polygon_height")
{
}

void PolygonHeightALM::configure(tue::Configuration config)
{
    if (config.readGroup("parameters"))
    {
        config.value("tolerance", tolerance_);
        config.value("min_cluster_size", min_cluster_size_);

        config.value("visualize", visualize_);

        std::cout << "Parameters polygon height association: \n" <<
        "- tolerance: " << tolerance_ << "\n" <<
        "- min_cluster_size: " << min_cluster_size_ << "\n" <<
        "- visualize: " << visualize_ << std::endl;

        config.endGroup();
    }
}

void PolygonHeightALM::process(const RGBDData& rgbd_data,
                               PointCloudMaskPtr& not_associated_mask,
                               const WorldModelConstPtr& world_model,
                               ALMResult& result)
{
    // First find the clusters
    profiler_.startTimer("find_euclidean_clusters");
    std::vector<PointCloudMaskPtr> clusters;
    helpers::ddp::findEuclideanClusters(rgbd_data.point_cloud, not_associated_mask, tolerance_, min_cluster_size_, clusters);
    not_associated_mask->clear();
    profiler_.stopTimer();

    //! 2) Association
    unsigned int i = 0;
    profiler_.startTimer("association");
    // Keep track of the entities that have an association
    std::map<UUID, std::vector<std::pair<PointCloudMaskPtr,ConvexHull2D> > > associated_entities;
    for(std::vector<PointCloudMaskPtr>::const_iterator it = clusters.begin(); it != clusters.end(); ++it )
    {
        const PointCloudMaskPtr& cluster = *it;

        //! Create the Polygon with Height
        ConvexHull2D polygon;
        helpers::ddp::get2DConvexHull(rgbd_data.point_cloud, *cluster, rgbd_data.sensor_pose, polygon);

        helpers::visualization::publishConvexHull2DVisualizationMarker(polygon, vis_marker_pub_, i, "bla");
        ++i;

        bool associated = false;
        UUID associated_id = "";

        for(WorldModel::const_iterator e_it = world_model->begin(); e_it != world_model->end(); ++e_it)
        {
            const EntityConstPtr& e = *e_it;
            if (e->shape())
                continue;

            // Multiple measurements per entity BUT 1 entity per measurement
            double overlap_factor;
            if ( helpers::ddp::polygonCollisionCheck(polygon, e->convexHull(), overlap_factor) )
            {
                associated = true;
                associated_id = e->id();
                break;
            }
        }

        if (associated)
            // Keep track of entities that have been associated
            associated_entities[associated_id].push_back(std::make_pair<PointCloudMaskPtr, ConvexHull2D>(cluster,polygon));
        else
            not_associated_mask->insert(not_associated_mask->end(), cluster->begin(), cluster->end());
    }
    profiler_.stopTimer();

    // Add the associated clusters
    for (std::map<UUID, std::vector<std::pair<PointCloudMaskPtr,ConvexHull2D> > >::iterator it = associated_entities.begin(); it != associated_entities.end(); ++it)
    {
        ConvexHull2D polygon;
        PointCloudMaskPtr pcl_mask(new PointCloudMask());
        for (std::vector<std::pair<PointCloudMaskPtr,ConvexHull2D> >::iterator cit = it->second.begin(); cit != it->second.end(); ++cit)
        {
            for (unsigned int i = 0; i < cit->first->size(); ++i)
            {
                pcl_mask->push_back(cit->first->at(i));
            }
            helpers::ddp::add2DConvexHull(cit->second, polygon);
        }

        // Create the measurement (For now based on one found convex hull, other info gets rejected)
        MeasurementPtr m(new Measurement(rgbd_data, pcl_mask, polygon));

        result.addAssociation(it->first, m);
    }

    pub_profile_.publish();
}

}
