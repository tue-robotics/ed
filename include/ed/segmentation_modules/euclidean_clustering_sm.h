#ifndef euclidean_clustering_sm_h_
#define euclidean_clustering_sm_h_

#include "ed/segmentation_modules/rgbd_seg_module.h"

namespace ed
{

class EuclideanClusteringSM : public RGBDSegModule
{

public:

    EuclideanClusteringSM();

    void process(const RGBDData& rgbd_data, std::vector<PointCloudMaskPtr>& segments);

    void configure(tue::Configuration config);

private:
    double tolerance_;
    int min_cluster_size_;

};

}

#endif
