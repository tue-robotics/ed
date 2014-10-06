#include "ed/io/filesystem/write.h"

#include "ed/measurement.h"
#include "ed/serialization/serialization.h"

#include <tue/serialization/output_archive.h>

#include <rgbd/serialization.h>

#include <geolib/serialization.h>

#include <fstream>

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

    // save sensor pose
    {
        std::string filename_pose = filename + ".poseMAP";
        std::ofstream f_out;
        f_out.open(filename_pose.c_str(), std::ifstream::binary);
        if (f_out.is_open())
        {
            f_out << msr.sensorPose();
        }
        else
        {
            std::cout << "Could not save to " << filename_pose << std::endl;
        }
    }

    return true;
}

}
