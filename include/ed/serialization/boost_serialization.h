#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include "ed/uuid.h"
#include "ed/entity.h"
#include "ed/world_model.h"
#include "ed/measurement.h"
#include "ed/types.h"
#include "ed/property_key_db.h"
#include "tue/config/map.h"
#include "geolib/Shape.h"

#include "ed/io/filesystem/read.h"
#include "ed/io/filesystem/write.h"

#include <iostream>
#include <pcl/io/pcd_io.h>

//serialization functions for use with boost::serialization

//note: make_nvp() is required for use with xml archives

namespace ed { // functions that require access to private members

    template <class Archive>
    void WorldModel::serialize(Archive & ar, const unsigned int /*version*/) {
        ar & make_nvp("revision_", revision_);
        ar & make_nvp("entity_map_", entity_map_);
        ar & make_nvp("entities_", entities_);
        ar & make_nvp("entity_revisions_", entity_revisions_);
        ar & make_nvp("entity_shape_revisions_", entity_shape_revisions_);
        ar & make_nvp("entity_empty_spots_", entity_empty_spots_);
        //ar & make_nvp("relations_", relations_); //relations don't seem to contain data
        ar & make_nvp("property_info_db_", property_info_db_);
    }

    template <class Archive>
    void Entity::serialize(Archive & ar, const unsigned int /*version*/) {
        ar & make_nvp("id_", id_);
        ar & make_nvp("revision_", revision_);
        ar & make_nvp("type_", type_);
        ar & make_nvp("types_", types_);
        ar & make_nvp("existence_prob_", existence_prob_);
        ar & make_nvp("last_update_timestamp_", last_update_timestamp_);
        ar & make_nvp("measurements_", measurements_);
        ar & make_nvp("best_measurement_", best_measurement_);
        ar & make_nvp("measurements_seq_", measurements_seq_);
        ar & make_nvp("shape_", shape_);
        ar & make_nvp("volumes_", volumes_);
        ar & make_nvp("shape_revision_", shape_revision_);
        ar & make_nvp("convex_hull_map_", convex_hull_map_);
        ar & make_nvp("convex_hull_new_", convex_hull_new_);
        ar & make_nvp("has_pose_", has_pose_);
        ar & make_nvp("config_", config_);
        ar & make_nvp("relations from_", relations_from_);
        ar & make_nvp("relations_to_", relations_to)_;
        ar & make_nvp("properties_", properties_);
        ar & make_nvp("flags_", flags_);
    }

    template<class Archive>
    void UUID::serialize(Archive & ar, const unsigned int /*version*/)
    {
        ar & make_nvp("id_", id_);
        ar & make_nvp("idx", idx);
    }
    
    /*template <class Archive>
    void Measurement::serialize(Archive & ar, const unsigned int version)
    {
        ar & make_nvp("rgbd_data_", rgbd_data_);
        ar & make_nvp("mask_", mask_);
        ar & make_nvp("image_mask_", image_mask_);
        ar & make_nvp("timestamp_", timestamp_);
        ar & make_nvp("seq_", seq_);
    }*/

    template <class Archive>
    void PropertyKeyDB::serialize(Archive & ar, const unsigned int /*version*/)
    {
        ar & make_nvp("name_to_info_", name_to_info_);
    }

}//namespace ed

namespace boost {
namespace serialization { // non-intrusive functions that only use public members

    //BOOST_CLASS_TRACKING(tue::config::Variant, track_never)//addresses to members of Variant cannot be stored properly, so no use trying to track them
    
    template <class Archive>
    void serialize(Archive & ar, tue::config::Variant & variant, const unsigned int version)
    {
        split_free(ar, variant, version);//use save and load functions
    }

    template <class Archive>
    void save(Archive & ar, tue::config::Variant & variant, const unsigned int /*version*/)
    {
        //important to write type first, so it can be read first
        if (variant.isString())
        {
            ar << make_nvp("type", "s");
            std::string s;
            variant.getValue(s);
            ar << make_nvp("value", s);
        }
        else if (variant.isInt())
        {
            ar << make_nvp("type", "i");
            int i;
            variant.getValue(i);
            ar << make_nvp("value", i);
        }
        else if (variant.isDouble())
        {
            ar << make_nvp("type", "d");
            double d;
            variant.getValue(d);
            ar << make_nvp("value", d);
        }
        else if (variant.isBoolean())
        {
            ar << make_nvp("type", "b");
            bool b;
            variant.getValue(b);
            ar << make_nvp("value", b);
        }
        else
        {
            ar << make_nvp("type", "?");
            //no use storing an empty value
        }
    }

    template <class Archive>
    void load(Archive & ar, tue::config::Variant & variant, const unsigned int /*version*/)
    {
        std::string type;
        ar >> make_nvp("type", type);
        //check which type is stored and read the appropriate type
        if (type == "s")
        {
            std::string s;
            ar >> make_nvp("value", s);
            variant = tue::config::Variant(s);
        }
        else if (type == "i")
        {
            int i;
            ar >> make_nvp("value", i);
            variant = tue::config::Variant(i);
        }
        else if (type == "d")
        {
            double d;
            ar >> make_nvp("value", d);
            variant = tue::config::Variant(d);
        }
        else if (type == "b")
        {
            bool b;
            ar >> make_nvp("value", b);
            variant = tue::config::Variant(b);
        }
        else
        {
            //value is not written so don't read it
            variant = tue::config::Variant();
        }
    }

    template <class Archive>
    void save(Archive & ar, ed::Measurement & measurement, const unsigned int /*version*/)
    {
        //requires a rewrite of measurement saving
    }

    template <class Archive>
    void serialize(Archive & ar, ed::MeasurementConvexHull & mchull, const unsigned int /*version*/)
    {
        ar & make_nvp("convex_hull", mchull.convex_hull);
        ar & make_nvp("pose", mchull.pose);
        ar & make_nvp("timestamp", mchull.timestamp);
    }

    template <class Archive>
    void serialize(Archive & ar, ed::ConvexHull & chull, const unsigned int /*version*/)
    {
        ar & make_nvp("points", chull.points);
        ar & make_nvp("edges", chull.edges);
        ar & make_nvp("normals", chull.normals);
        ar & make_nvp("z_min", chull.z_min);
        ar & make_nvp("z_max", chull.z_max);
        ar & make_nvp("area", chull.area);
        ar & make_nvp("complete", chull.complete);
    }

    template <class Archive>
    void serialize(Archive & ar, tue::config::Data & data, const unsigned int /*version*/)
    {
        ar & make_nvp("label_to_name", data.label_to_name);
        ar & make_nvp("name_to_label", data.name_to_label);
        ar & make_nvp("nodes", data.nodes);
        ar & make_nvp("parents", data.parents);
        ar & make_nvp("right_siblings", data.right_siblings);
        ar & make_nvp("source_", data.source_);
    }

    template <class Archive>
    void serialize(Archive & ar, tue::config::DataPointer & datapointer, const unsigned int /*version*/)
    {
        ar & make_nvp("data", datapointer.data);
        ar & make_nvp("idx", datapointer.idx);
    }

    template <class Archive>
    void serialize(Archive & ar, tue::config::DataConstPointer & datapointer, const unsigned int /*version*/)
    {
        ar & make_nvp("data", datapointer.data);
        ar & make_nvp("idx", datapointer.idx);
    }

    template <class Archive>
    void serialize(Archive & ar, tue::config::Map & map, const unsigned int version)
    {
        split_free(ar, map, version);//use save and load functions
    }

    template <class Archive>
    void save(Archive & ar, tue::config::Map & map, const unsigned int /*version*/)
    {
        ar << make_nvp("name", map.name());
        ar << make_nvp("map_", map.map_);
        ar << make_nvp("values", map.values);
    }

    template <class Archive>
    void load(Archive & ar, tue::config::Map & map, const unsigned int /*version*/)
    {
        tue::Label name;
        ar >> make_nvp("name", name);
        ar >> make_nvp("map_", map.map_);
        ar >> make_nvp("values", map.values);
        map.setName(name);
        ar.reset_object_address(&(map.name()), &name);
    }

    template <class Archive>
    void serialize(Archive & ar, ed::Property & property, const unsigned int /*version*/)
    {
        ar & make_nvp("entry", property.entry);
        ar & make_nvp("revision", property.revision);
        ar & make_nvp("value", property.value);
    }

    template <class Archive>
    void serialize(Archive & ar, ed::PropertyKeyDBEntry & property_key_db_entry, const unsigned int /*version*/)
    {
        ar & make_nvp("name", property_key_db_entry.name);
        ar & make_nvp("idx", property_key_db_entry.idx);
        //ar & make_nvp("info", property_key_db_entry.info); //property_info seems incomplete
    }

    template <class Archive, class T>
    void serialize(Archive & ar, geo::Transform3T<T> & pose, const unsigned int /*version*/)
    {
        ar & make_nvp("t", pose.t);
        ar & make_nvp("R", pose.R);
    }
    
    template <class Archive, class T>
    void serialize(Archive & ar, geo::Mat3T<T> & mat, const unsigned int /*version*/)
    {
        ar & make_nvp("xx", mat.xx);
        ar & make_nvp("xy", mat.xy);
        ar & make_nvp("xz", mat.xz);
        ar & make_nvp("yx", mat.yx);
        ar & make_nvp("yy", mat.yy);
        ar & make_nvp("yz", mat.yz);
        ar & make_nvp("zx", mat.zx);
        ar & make_nvp("zy", mat.zy);
        ar & make_nvp("zz", mat.zz);
    }

    template <class Archive, class T>
    void serialize(Archive & ar, geo::Vec3T<T> & vec, const unsigned int /*version*/)
    {
        ar & make_nvp("x", vec.x);
        ar & make_nvp("y", vec.y);
        ar & make_nvp("z", vec.z);
    }

    template <class Archive, class T>
    void serialize(Archive & ar, geo::Vec2T<T> & vec, const unsigned int /*version*/)
    {
        ar & make_nvp("x", vec.x);
        ar & make_nvp("y", vec.y);
    }

    template <class Archive>
    void serialize(Archive & ar, geo::Shape & shape, const unsigned int /*version*/)
    {
        ar & make_nvp("mesh", shape.getMesh());
    }

    template <class Archive>
    void serialize(Archive & ar, geo::Mesh & mesh, const unsigned int version)
    {
        split_free(ar, mesh, version);//use save and load functions
    }

    template <class Archive>
    void save(Archive, geo::Mesh & mesh, const unsigned int /*version*/)
    {
        ar << make_nvp("points", mesh.getPoints());
        ar << make_nvp("triangleIs", mesh.getTriangleIs());
    }

    template <class Archive>
    void load(Archive, geo::Mesh & mesh, const unsigned int /*version*/)
    {
        std::vector<geo::Vector3> points;
        std::vector<geo::TriangleI> triangleIs;
        ar >> make_nvp("points", points);
        ar >> make_nvp("triangleIs", triangleIs);
        for (std::vector<geo::Vector3>::iterator it = points.begin(); it = points.end(); it++)
        {
            mesh.addPoint(*it);
        }
        for (std::vector<geo::TriangleI>::iterator it = triangleIs.begin(); it = triangleIs.end(); it++)
        {
            mesh.addTriangle(*it);
        }
        ar.reset_object_address(&(mesh.getPoints()), &points);
        ar.reset_object_address(&(mesh.getTriangleIs()), &triangleIs);
    }

    template <class Archive>
    void serialize(Archive & ar, geo::TriangleI & triangleI, const unsigned int /*version*/)
    {
        ar & make_nvp("i1_", triangleI.i1_);
        ar & make_nvp("i2_", triangleI.i2_);
        ar & make_nvp("i3_", triangleI.i3_);
    }

    /*template <class Archive>
    void serialize(Archive & ar, ed::RGBDData & rgbdData, const unsigned int version)
    {
        ar & make_nvp("image", rgbdData.image);
        ar & make_nvp("sensor_page", rgbdData.sensor_pose);
        ar & make_nvp("point_cloud", rgbdData.point_cloud);
        ar & make_nvp("point_cloud_with_normals", rgbdData.point_cloud_with_normals);
        ar & make_nvp("point_cloud_to_pixels_mapping", rgbdData.point_cloud_to_pixels_mapping);
    }*/

    /*template <class Archive>
    void serialize(Archive & ar, rgbd::Image & image, const unsigned int version)
    {
        ar & image.getCameraModel();//this part is a lot of work, consider saving images to a file and storing filename in archive
        ar & image.getDepthImage();
        ar & image.getFrameId();
        ar & image.getRGBImage();
        ar & image.getTimestamp();
    }*/

    /*template <class Archive>
    void save(Archive & ar, rgbd::Image & image, const unsigned int version)
    {
        static unsigned int image_counter;
        std::string image_filename = "DepthImages/Image_" + image_counter + ".rgbd";

    }*/

    template <class Archive>
    void serialize(Archive & ar, cv::Mat & mat, const unsigned int /*version*/)
    {
        ar & make_nvp("flags", mat.flags);
        ar & make_nvp("dims", mat.dims);
        ar & make_nvp("cols", mat.cols);
        ar & make_nvp("rows", mat.rows);
        ar & make_nvp("data", mat.data);
    }//check if this works

    template <class Archive, class T>
    void serialize(Archive & ar, cv::Mat_<T> & mat, const unsigned int /*version*/)
    {
        ar & make_nvp("flags", mat.flags);
        ar & make_nvp("dims", mat.dims);
        ar & make_nvp("cols", mat.cols);
        ar & make_nvp("rows", mat.rows);
        ar & make_nvp("data", mat.data);
    }//check if this works

    /*template <class Archive>
    void serialize(Archive & ar, image_geometry::PinholeCameraModel & cameraModel, const unsigned int version)
    {
        ar & cameraModel.cameraInfo();
        ar & cameraModel.intrinsicMatrix();
        ar & cameraModel.distortionCoeffs();
        ar & cameraModel.rotationMatrix();
        ar & cameraModel.projectionMatrix();
        ar & cameraModel.fullIntrinsicMatrix();
        ar & cameraModel.fullProjectionMatrix();
    }*/

    /*template <class Archive, typename PointT>
    void save(Archive & ar, pcl::PointCloud<PointT> & pointcloud, const unsigned int version)
    {
        //save pointcloud as pcd file and store file location

        //make sure each pointcloud has a unique filename
        static unsigned int pointcloud_counter;//might consider using UUID, however the old pointclouds would have to be removed
        std::string pointcloud_filename = "Pointclouds/Pointcloud_" + pointcloud_counter + ".pcd"
        pointcloud_counter++;
        
        //save pointcloud
        pcl::io::savePCDFileASCII(pointcloud_filename, pointcloud);
        ar << make_nvp("filename", pointcloud_filename);
    }

    template <class Archive, typename PointT>
    void load(Archive & ar, pcl::PointCloud<PointT> & pointcloud, const unsigned int version)
    {
        //read filename
        std::string pointcloud_filename;
        ar >> make_nvp("filename", pointcloud_filename);
        //read pointcloud
        pcl::io::loadPCDFile(pointcloud_filename, pointcloud);

    }*/

    template <class Archive, class T>
    void serialize(Archive & ar, boost::circular_buffer<T> & buffer, const unsigned int version)
    {
        split_free(ar, buffer, version);//use save and load functions
    }

    template <class Archive, class T>
    void save(Archive & ar, boost::circular_buffer<T> & buffer, const unsigned int /*version*/)
    {
        ar << make_nvp("capacity", buffer.capacity());
        ar << make_nvp("size", buffer.size());
        for (unsigned int i = 0; i < buffer.size(); i++)
        {
            ar << make_nvp("value_" + i, buffer[i]);
        }
    }

    template <class Archive, class T>
    void load(Archive & ar, boost::circular_buffer<T> & buffer, const unsigned int /*version*/)
    {
        typename boost::circular_buffer<T>::capacity_type capacity;
        typename boost::circular_buffer<T>::size_type size;

        ar >> make_nvp("capacity", capacity);
        ar >> make_nvp("size", size);
        buffer.set_capacity(capacity);
        buffer.clear();
        T value;
        for (unsigned int i = 0; i < size; i++)
        {
            ar >> make_nvp("value_" + i, value);
            buffer.push_back(value);
            ar.reset_object_address(&(buffer.back()), &value)//whereas capacity and size do not require accurate tracking, the stored values must be tracked properly(you might need a pointer to one of the stored values)
        }
    }
    
    

    //#TODO ed::Measurement, ed::RelationConstPointer
}//namespace serialization
}//namespace boost