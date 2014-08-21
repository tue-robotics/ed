#include "ed/association_localization_modules/polygon_height_alm.h"

#include "ed/helpers/visualization.h"
#include "ed/helpers/depth_data_processing.h"

#include "ed/entity.h"
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
        config.value("cell_size", cell_size_);
        config.value("max_range", max_range_);
        config.value("tolerance", tolerance_);
        config.value("min_cluster_size", min_cluster_size_);

        config.value("visualize", visualize_);

        std::cout << "Parameters polygon height association: \n" <<
        "- cell_size: " << cell_size_ << "\n" <<
        "- max_range: " << max_range_ << "\n" <<
        "- tolerance: " << tolerance_ << "\n" <<
        "- min_cluster_size: " << min_cluster_size_ << "\n" <<
        "- visualize: " << visualize_ << std::endl;

        config.endGroup();
    }
}

void PolygonHeightALM::process(const RGBDData& rgbd_data,
                               PointCloudMaskPtr& not_associated_mask,
                               std::map<UUID, EntityConstPtr>& entities)
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
    std::set<UUID> associated_entities;
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
        unsigned int max_measurement_seq = 0;

        for(std::map<UUID, EntityConstPtr>::const_iterator e_it = entities.begin(); e_it != entities.end(); ++e_it)
        {
            const EntityConstPtr& e = e_it->second;
            if (e->shape())
                continue;

            // Multiple measurements per entity BUT 1 entity per measurement
            double overlap_factor;
            if ( helpers::ddp::polygonCollisionCheck(polygon, e->convexHull(), overlap_factor) )
            {
                associated = true;
                if (e->measurementSeq() > max_measurement_seq)
                    associated_id = e->id();
            }
        }

        // If this cluster did not associate, add it to 'not associated mask'
        if (associated)
        {
            // Create the measurement (For now based on one found convex hull, other info gets rejected)
            MeasurementPtr m(new Measurement(rgbd_data, cluster, polygon));

            // Make a copy of the entity such that we can add the associated measurement
            EntityPtr e_updated(new Entity(*entities[associated_id]));

            // Add measurement to entity
            e_updated->addMeasurement(m);

            // Add updated entity to the map
            entities[associated_id] = e_updated;

            // Keep track of entities that have been associated
            associated_entities.insert(associated_id);
        }
        else
        {
            not_associated_mask->insert(not_associated_mask->end(), cluster->begin(), cluster->end());
        }
    }
    profiler_.stopTimer();

    //! 3. Collect entities in view (based on center point)
    std::vector<UUID> entities_in_view_not_associated;
    profiler_.startTimer("get_entities_in_view");
    for (std::map<UUID, EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const EntityConstPtr& e = it->second;

        if (!e->shape()) //! if it has a shape
        {
            bool in_frustrum, object_in_front;
            if (helpers::ddp::inView(rgbd_data.image, rgbd_data.sensor_pose, e->convexHull().center_point, 2.5, in_frustrum, object_in_front))
            {
                if (associated_entities.find(it->first) == associated_entities.end()) //! if no association found with this object
                {
                    entities_in_view_not_associated.push_back(it->first);
                }
            }
        }
    }
    profiler_.stopTimer();

    //! 5. Remove entities that do not have an association
    profiler_.startTimer("clearing");
    for (std::vector<UUID>::const_iterator it = entities_in_view_not_associated.begin(); it != entities_in_view_not_associated.end(); ++it)
        entities.erase(*it);
    profiler_.stopTimer();

    pub_profile_.publish();
}

}
