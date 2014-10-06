#ifndef point_normal_alm_h_
#define point_normal_alm_h_

#include "ed/association_localization_modules/rgbd_al_module.h"

#include <ros/publisher.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace ed
{

// Define a new point representation for the association
class AssociationPR : public pcl::PointRepresentation <pcl::PointNormal>
{
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

    public:
        AssociationPR ()
        {
            // Define the number of dimensions
            nr_dimensions_ = 6;
        }

        // Override the copyToFloatArray method to define our feature vector
        virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
        {
            // < x, y, z, curvature >
            out[0] = p.x;
            out[1] = p.y;
            out[2] = p.z;
            out[3] = p.normal_x;
            out[4] = p.normal_y;
            out[5] = p.normal_z;
//            out[3] = p.curvature;
        }
};

class PointNormalALM : public RGBDALModule
{

public:

    PointNormalALM();

    void process(const RGBDData& rgbd_data,
                 PointCloudMaskPtr& not_associated_mask,
                 std::map<UUID, EntityConstPtr>& entities);

    void configure(tue::Configuration config);

protected:

    pcl::KdTreeFLANN<pcl::PointNormal>::Ptr tree_;
    AssociationPR point_representation_, point_representation2_;

    //! tunable params
    float position_weight_;
    float normal_weight_;
    float association_correspondence_distance_;
    int render_width_;
    float render_max_range_;
    float render_voxel_size_;
    int normal_k_search_;

};

}

#endif
