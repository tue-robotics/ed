#include "ed/io/filesystem/write.h"

#include "ed/measurement.h"
#include "ed/serialization/serialization.h"

#include <tue/serialization/output_archive.h>

#include <rgbd/serialization.h>

#include <geolib/serialization.h>

#include <fstream>

#include "ed/io/json_writer.h"

#include <ed/logging.h>

#include "ed/entity.h"

#include <tue/filesystem/path.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

bool write(const std::string& filename, const Measurement& msr)
{
    // save image
    {
        std::string filename_image = filename + ".rgbd";
        std::ofstream f_out;
        f_out.open(filename_image.c_str(), std::ifstream::binary);
        if (f_out.is_open())
        {
            tue::serialization::OutputArchive a_out(f_out);
            rgbd::serialize(*msr.image(), a_out);
        }
        else
        {
            std::cout << "Could not save to " << filename_image << std::endl;
        }
    }

    // save mask
    {
        std::string filename_mask = filename + ".mask";
        std::ofstream f_out;
        f_out.open(filename_mask.c_str(), std::ifstream::binary);
        if (f_out.is_open())
        {
            tue::serialization::OutputArchive a_out(f_out);
            ed::serialize(msr.imageMask(), a_out);
        }
        else
        {
            std::cout << "Could not save to " << filename_mask << std::endl;
        }
    }

//    // save sensor pose
//    {
//        std::string filename_pose = filename + ".poseMAP";
//        std::ofstream f_out;
//        f_out.open(filename_pose.c_str(), std::ifstream::binary);
//        if (f_out.is_open())
//        {
//            f_out << msr.sensorPose();
//        }
//        else
//        {
//            std::cout << "Could not save to " << filename_pose << std::endl;
//        }
//    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

namespace
{

void write(const std::string& name, const geo::Vector3& v, io::Writer& w)
{
    w.writeGroup(name);
    w.writeValue("x", v.x);
    w.writeValue("y", v.y);
    w.writeValue("z", v.z);
    w.endGroup();
}

void write(const std::string& name, const geo::Matrix3& m, io::Writer& w)
{
    w.writeGroup(name);
    w.writeValue("xx", m.xx);
    w.writeValue("xy", m.xy);
    w.writeValue("xz", m.xz);
    w.writeValue("yx", m.yx);
    w.writeValue("yy", m.yy);
    w.writeValue("yz", m.yz);
    w.writeValue("zx", m.zx);
    w.writeValue("zy", m.zy);
    w.writeValue("zz", m.zz);
    w.endGroup();
}

void write(const std::string& name, const geo::Pose3D& p, io::Writer& w)
{
    w.writeGroup(name);
    write("t", p.t, w);
    write("R", p.R, w);
    w.endGroup();
}

}

// ----------------------------------------------------------------------------------------------------

bool write(const std::string& filename, const Entity& e)
{
    std::string filename_ext = filename + ".json";
    std::ofstream f_out;
    f_out.open(filename_ext.c_str());

    if (!f_out.is_open())
    {
        ed::log::error() << "Could not save to '" << filename_ext << "'" << std::endl;
        return false;
    }

    ed::io::JSONWriter w(f_out);

    w.writeValue("id", e.id().str());
    w.writeValue("type", e.type());

    // Convex hull
    const ed::ConvexHull& chull = e.convexHull();
    if (!chull.points.empty())
    {
        w.writeGroup("convex_hull");
        w.writeValue("z_min", chull.z_min);
        w.writeValue("z_max", chull.z_max);

        w.writeArray("points");
        for(unsigned int i = 0; i < chull.points.size(); ++i)
        {
            const geo::Vec2f& p = chull.points[i];

            w.addArrayItem();
            w.writeValue("x", p.x);
            w.writeValue("y", p.y);
            w.endArrayItem();
        }

        w.endArray();

        w.endGroup();
    }

    // Pose
    write("pose", e.pose(), w);

    // RGBD Measurement
    ed::MeasurementConstPtr msr = e.lastMeasurement();
    if (msr)
    {
        w.writeGroup("rgbd_measurement");

        // Get filename without path
        std::string base_filename = tue::filesystem::Path(filename).filename();

        w.writeValue("image_file", base_filename + ".rgbd");
        w.writeValue("mask_file", base_filename + ".mask");
        write("sensor_pose", msr->sensorPose(), w);

        write(filename, *msr);

        w.endGroup();
    }

    w.finish();

    return true;
}

}
