#include "ed/io/filesystem/write.h"

#include "ed/entity.h"
#include "ed/io/json_writer.h"
#include "ed/logging.h"
#include "ed/measurement.h"
#include "ed/serialization/serialization.h"
#include "ed/types.h"

#include <filesystem>
#include <iostream>
#include <ostream>
#include <string>
#include <tue/serialization/output_archive.h>

#include <rgbd/serialization.h>

#include <fstream>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

bool write(const std::string& filename, const Measurement& msr)
{
    // save image
    {
        std::string const filename_image = filename + ".rgbd";
        std::ofstream f_out;
        f_out.open(filename_image.c_str(), std::ifstream::binary);
        if (f_out.is_open())
        {
            tue::serialization::OutputArchive a_out(f_out);
            rgbd::serialize(*msr.image(), a_out);
        }
        else
        {
            std::cout << "Could not save to " << filename_image << '\n';
        }
    }

    // save mask
    {
        std::string const filename_mask = filename + ".mask";
        std::ofstream f_out;
        f_out.open(filename_mask.c_str(), std::ifstream::binary);
        if (f_out.is_open())
        {
            tue::serialization::OutputArchive a_out(f_out);
            ed::serialize(msr.imageMask(), a_out);
        }
        else
        {
            std::cout << "Could not save to " << filename_mask << '\n';
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool write(const std::string& filename, const Entity& e)
{
    std::string const filename_ext = filename + ".json";
    std::ofstream f_out;
    f_out.open(filename_ext.c_str());

    if (!f_out.is_open())
    {
        ed::log::error() << "Could not save to '" << filename_ext << "'" << '\n';
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
        ed::serialize(chull, w);
        w.endGroup();
    }

    // Pose
    if (e.hasPose())
    {
        w.writeGroup("pose");
        ed::serialize(e.pose(), w);
        w.endGroup();
    }

    // RGBD Measurement
    ed::MeasurementConstPtr const msr = e.lastMeasurement();
    if (msr)
    {
        w.writeGroup("rgbd_measurement");

        // Get filename without path
        std::string const base_filename = std::filesystem::path(filename).filename().string();

        w.writeValue("image_file", base_filename + ".rgbd");
        w.writeValue("mask_file", base_filename + ".mask");

        w.writeGroup("sensor_pose");
        ed::serialize(msr->sensorPose(), w);
        w.endGroup();

        write(filename, *msr);

        w.endGroup();
    }

    w.finish();

    return true;
}

} // namespace ed
