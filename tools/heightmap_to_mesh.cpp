#include "../src/models/shape_loader_private.h"

#include <cstdlib>
#include <geolib/datatypes.h>
#include <geolib/io/export.h>
#include <geolib/Shape.h> // IWYU pragma: keep  // complete geo::Shape type needed for dereference below
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

int main(int argc, char** argv)
{

    // Parse command-line arguments
    if (argc < 3 || argc > 7)
    {
        std::cout << "Usage: ed_heightmap_to_mesh INPUT_IMAGE OUTPUT_FILE RESOLUTION [BLOCK_HEIGHT] [ORIGIN_X ORIGIN_Y]"
                  << '\n';
        return 1;
    }

    std::string const input_file = argv[1];
    std::string const output_file = argv[2];

    double resolution = 0.2;
    if (argc > 2)
    {
        resolution = atof(argv[3]);
    }

    double block_height = 1;
    if (argc > 3)
    {
        block_height = atof(argv[4]);
    }

    double origin_x = 0;
    double origin_y = 0;
    if (argc > 5)
    {
        if (argc < 7)
        {
            std::cout << "ORIGIN_X and ORIGIN_Y are optional, but shoud be provided together" << '\n';
            return 1;
        }
        origin_x = atof(argv[5]);
        origin_y = atof(argv[6]);
    }

    // Call shape loader. This will generate a mesh from the file
    std::stringstream error;
    geo::ShapePtr const shape = ed::models::getHeightMapShape(
        input_file, geo::Vec3(origin_x, origin_y, 0), block_height, resolution, resolution, false, error);

    if (!shape)
    {
        std::cout << "could not load heightmap: " << input_file << '\n' << error.str() << '\n';
        return 1;
    }

    if (!geo::io::writeMeshFile(output_file, *shape))
    {
        std::cout << "Could not convert loaded shape to mesh file: " << output_file << '\n';
        return 1;
    }

    std::cout << "Succesfully converted: '" << input_file << "' to '" << output_file << "'. With "
              << shape->getMesh().getPoints().size() << " points and " << shape->getMesh().getTriangleIs().size()
              << " triangles." << '\n';

    return 0;
}
