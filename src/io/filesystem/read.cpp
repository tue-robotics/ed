#include "ed/io/filesystem/read.h"

#include "ed/measurement.h"
#include "ed/serialization/serialization.h"

#include <rgbd/Image.h>
#include <rgbd/serialization.h>

#include <tue/serialization/input_archive.h>

#include <fstream>

#include "ed/io/json_reader.h"

#include "ed/entity.h"
#include "ed/update_request.h"
#include "ed/logging.h"
#include "ed/convex_hull_calc.h"

#include <tue/filesystem/path.h>

namespace ed
{

namespace
{

rgbd::ImagePtr readRGBDImage(const std::string& filename)
{
    rgbd::ImagePtr image(new rgbd::Image);

    std::ifstream f_in;
    f_in.open(filename.c_str(), std::ifstream::binary);

    if (!f_in.is_open())
    {
        std::cout << "Could not open '" << filename << "'." << std::endl;
        return rgbd::ImagePtr();
    }

    tue::serialization::InputArchive a_in(f_in);
    rgbd::deserialize(a_in, *image);

    return image;
}

bool readImageMask(const std::string& filename, ed::ImageMask& mask)
{
    std::ifstream f_in;
    f_in.open(filename.c_str(), std::ifstream::binary);

    if (!f_in.is_open())
    {
        std::cout << "Could not open '" << filename << "'." << std::endl;
        return false;
    }

    tue::serialization::InputArchive a_in(f_in);
    ed::deserialize(a_in, mask);

    return true;
}

}

// ----------------------------------------------------------------------------------------------------

bool read(const std::string& filename, Measurement& msr)
{
    // Read image
    rgbd::ImagePtr image = readRGBDImage(filename + ".rgbd");

    // Read mask
    ed::ImageMask mask;
    readImageMask(filename + ".mask", mask);

    msr = Measurement(image, mask, geo::Pose3D::identity()); // TODO: read pose

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool readEntity(const std::string& filename, UpdateRequest& req)
{
    std::ifstream f_in;
    f_in.open(filename.c_str());

    if (!f_in.is_open())
    {
        std::cout << "Could not open '" << filename << "'." << std::endl;
        return false;
    }

    std::stringstream buffer;
    buffer << f_in.rdbuf();
    std::string str = buffer.str();

    io::JSONReader r(str.c_str());

    // ID
    ed::UUID id;
    std::string id_str;
    if (r.readValue("id", id_str))
        id = id_str;
    else
        id = Entity::generateID();

    // Type
    std::string type;
    if (r.readValue("type", type))
        req.setType(id, type);

    // Pose
    geo::Pose3D pose;
    if (r.readGroup("pose"))
    {
        if (ed::deserialize(r, pose))
            req.setPose(id, pose);
        r.endGroup();
    }

    // Convex hull
    if (r.readGroup("convex_hull"))
    {
        ConvexHull chull;
        r.readValue("z_min", chull.z_min);
        r.readValue("z_max", chull.z_max);

        if (r.readArray("points"))
        {
            while(r.nextArrayItem())
            {
                chull.points.push_back(geo::Vec2f());
                geo::Vec2f& p = chull.points.back();
                r.readValue("x", p.x);
                r.readValue("y", p.y);
            }
            r.endArray();
        }

        ed::convex_hull::calculateEdgesAndNormals(chull);
        ed::convex_hull::calculateArea(chull);

        ed::log::warning() << "ed::readEntity: convex hull timestamp is set to 0." << std::endl;
        req.setConvexHullNew(id, chull, pose, 0);

        r.endGroup();
    }

    // RGBD measurement
    if (r.readGroup("rgbd_measurement"))
    {
        std::string rgbd_filename, mask_filename;
        if (r.readValue("image_file", rgbd_filename) && r.readValue("mask_file", mask_filename))
        {
            std::string base_path = tue::filesystem::Path(filename).parentPath().string();

            rgbd::ImagePtr image = readRGBDImage(base_path + "/" + rgbd_filename);

            // Read mask
            ed::ImageMask mask;
            readImageMask(base_path + "/" + mask_filename, mask);

            geo::Pose3D sensor_pose;
            if (r.readGroup("sensor_pose"))
            {
                ed::deserialize(r, sensor_pose);
                r.endGroup();
            }
            else
            {
                log::error() << "Could not read sensor pose from rgbd measurement" << std::endl;
                sensor_pose = geo::Pose3D::identity();
            }

            MeasurementPtr msr(new Measurement(image, mask, sensor_pose));

            req.addMeasurement(id, msr);
        }

        r.endGroup();
    }

    return true;
}

}
