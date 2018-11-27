// Shape loader, contains heightmap meshing algorithm
#include "../src/models/shape_loader.h"

// Visualization
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Shape traversal
#include <geolib/Shape.h>

// Config settings
#include <tue/config/writer.h>

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc <= 1)
    {
        std::cout << "Please provide a heightmap image file (e.g., pgm)" << std::endl;
        return 0;
    }

    std::string image_filename = argv[1];

    // Read input image
    cv::Mat viz = cv::imread(image_filename);

    if (!viz.data)
    {
        std::cout << "Could not load file." << std::endl;
        return 1;
    }

    // Set some default config the shape loader needs
    tue::config::Writer w;
    w.setValue("path", image_filename);
    w.setValue("origin_x", 0);
    w.setValue("origin_y", 0);
    w.setValue("origin_z", 0);
    w.setValue("resolution", 1);
    w.setValue("blockheight", 0);

    tue::config::Reader cfg(w.data()); // Wrap config in reader

    std::map<std::string, geo::ShapePtr> shape_cache; // necessary for call, not used

    // Call shape loader. This will generate a mesh from the file
    std::stringstream error;
    geo::ShapePtr shape = ed::models::loadShape("", cfg, shape_cache, error);

    if (!shape)
    {
        std::cout << "Could not generate mesh from file:" << std::endl;
        std::cout << error.str() << std::endl;
        return 1;
    }

    const std::vector<geo::TriangleI>& triangles = shape->getMesh().getTriangleIs();
    const std::vector<geo::Vector3>& vertices = shape->getMesh().getPoints();

    // Display number of vertices and triangles
    std::cout << vertices.size() << " vertices" << std::endl;
    std::cout << triangles.size() << " triangles" << std::endl;

    // Visualize triangles
    for(std::vector<geo::TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it)
    {
        const geo::TriangleI& t = *it;

        geo::Vector3 v1 = vertices[t.i1_];
        geo::Vector3 v2 = vertices[t.i2_];
        geo::Vector3 v3 = vertices[t.i3_];

        v1.y = viz.rows - v1.y;
        v2.y = viz.rows - v2.y;
        v3.y = viz.rows - v3.y;

        cv::line(viz, cv::Point(v1.x, v1.y), cv::Point(v2.x, v2.y), cv::Scalar(0, 0, 255), 1);
        cv::line(viz, cv::Point(v2.x, v2.y), cv::Point(v3.x, v3.y), cv::Scalar(0, 0, 255), 1);
        cv::line(viz, cv::Point(v3.x, v3.y), cv::Point(v1.x, v1.y), cv::Scalar(0, 0, 255), 1);
    }

    cv::imshow("image", viz);
    cv::waitKey();

    return 0;
}
