#ifndef polygon_height_alm_h_
#define polygon_height_alm_h_

#include "ed/association_localization_modules/rgbd_al_module.h"

#include <ros/publisher.h>

namespace ed
{

class PolygonHeightALM : public RGBDALModule
{

public:

    PolygonHeightALM();

    void process(const RGBDData& rgbd_data,
                 PointCloudMaskPtr& not_associated_mask,
                 std::map<UUID, EntityConstPtr>& entities);

    void configure(tue::Configuration config);

protected:

    /// Tunable params
    double cell_size_;
    double tolerance_;
    int min_cluster_size_;

};

}

#endif
