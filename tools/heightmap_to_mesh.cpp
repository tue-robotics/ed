#include "../src/models/shape_loader_private.h"
#include <geolib/Exporter.h>
//#include <geolib/Mesh.h>
//#include <geolib/Triangle.h>

int main(int argc, char **argv) {

    // Parse command-line arguments
    if (argc < 3 || argc > 7) {
        std::cout << "Usage: ed_heightmap_to_mesh INPUT_IMAGE OUTPUT_FILE RESOLUTION [BLOCK_HEIGHT] [ORIGIN_X ORIGIN_Y]" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    double resolution = 0.2;
    if (argc > 2) {
        resolution = atof(argv[3]);
    }

    double block_height = 1;
    if (argc > 3) {
        block_height = atof(argv[4]);
    }

    double origin_x = 0, origin_y = 0;
    if (argc > 5) {
        if (argc < 7) {
            std::cout << "ORIGIN_X and ORIGIN_Y are optional, but shoud be provided together" << std::endl;
            return 1;
        }
        origin_x = atof(argv[5]);
        origin_y = atof(argv[6]);
    }

    // Call shape loader. This will generate a mesh from the file
    std::stringstream error;
    geo::ShapePtr shape = ed::models::getHeightMapShape(input_file, geo::Vec3(origin_x, origin_y, 0), block_height,
                                                        resolution, resolution, false, error);

    if(!shape)
    {
        std::cout << "could not load heightmap: " << input_file << std::endl << error.str() << std::endl;
        return 1;
    }

    geo::Exporter exp;
    if (!exp.writeMeshFile(output_file, *shape))
    {
        std::cout << "Could not convert loaded shape to mesh file: " << output_file << std::endl;
        return 1;
    }

    std::cout << "Succesfully converted: '" << input_file << "' to '" << output_file <<"'. With " <<
                 shape->getMesh().getPoints().size() << " points and " << shape->getMesh().getTriangleIs().size() <<
                 " triangles." << std::endl;

    return 0;
}
