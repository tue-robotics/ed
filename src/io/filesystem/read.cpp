#include "ed/io/filesystem/read.h"

#include "ed/measurement.h"
#include "ed/serialization/serialization.h"

#include <rgbd/Image.h>
#include <rgbd/serialization.h>

#include <tue/serialization/input_archive.h>

#include <fstream>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

bool read(const std::string& filename, Measurement& msr)
{

    std::string filename_image = filename + ".rgbd";
    std::string filename_mask = filename + ".mask";

    // - - - - - - - - - - - - - - - read image - - - - - - - - - - - - - - -

    rgbd::ImagePtr image(new rgbd::Image);

    {
        std::ifstream f_in;
        f_in.open(filename_image.c_str(), std::ifstream::binary);

        if (!f_in.is_open())
        {
            std::cout << "Could not open '" << filename_image << "'." << std::endl;
            return false;
        }

        tue::serialization::InputArchive a_in(f_in);
        rgbd::deserialize(a_in, *image);
    }

    // - - - - - - - - - - - - - - - read mask - - - - - - - - - - - - - - -

    ed::ImageMask mask;

    {
        std::ifstream f_in;
        f_in.open(filename_mask.c_str(), std::ifstream::binary);

        if (!f_in.is_open())
        {
            std::cout << "Could not open '" << filename_mask << "'." << std::endl;
            return false;
        }

        tue::serialization::InputArchive a_in(f_in);
        ed::deserialize(a_in, mask);
    }

    msr = Measurement(image, mask, geo::Pose3D::identity()); // TODO: read pose

    return true;
}

}
