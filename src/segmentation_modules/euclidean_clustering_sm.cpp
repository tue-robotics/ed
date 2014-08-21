#include "ed/segmentation_modules/euclidean_clustering_sm.h"

#include "ed/helpers/depth_data_processing.h"
#include "ed/helpers/visualization.h"
#include "ed/mask.h"

namespace ed
{

EuclideanClusteringSM::EuclideanClusteringSM() : RGBDSegModule("euclidean_clustering")
{

}

void EuclideanClusteringSM::configure(tue::Configuration config)
{
    if (config.readGroup("parameters"))
    {
        config.value("tolerance", tolerance_);
        config.value("min_cluster_size", min_cluster_size_);

        std::cout << "Parameters euclidean clustering: \n" <<
        "- tolerance: " << tolerance_ << "\n" <<
        "- min_cluster_size: " << min_cluster_size_ << std::endl;

        config.endGroup();
    }
}

void EuclideanClusteringSM::process(const RGBDData& rgbd_data, std::vector<PointCloudMaskPtr>& segments)
{
    // Copy and clear segments
    std::vector<PointCloudMaskPtr> old_segments = segments;
    segments.clear();

    // Loop over all segments and perform euclidean cluster segmentation per segment
    for (std::vector<PointCloudMaskPtr>::const_iterator it = old_segments.begin(); it != old_segments.end(); ++it) {

        const PointCloudMaskPtr& old_seg = *it;

        // Get clusters from pcl
        std::vector<PointCloudMaskPtr> clusters;
        helpers::ddp::findEuclideanClusters(rgbd_data.point_cloud, old_seg, tolerance_, min_cluster_size_, clusters);

        // Iter over clusters and add segments
        for (std::vector<PointCloudMaskPtr>::const_iterator cit = clusters.begin(); cit != clusters.end(); ++cit)
            segments.push_back(*cit);
    }
}

}
