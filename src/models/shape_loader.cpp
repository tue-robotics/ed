#include "shape_loader.h"

#include <tue/filesystem/path.h>
#include <ros/package.h>
#include <fstream>

#include <geolib/serialization.h>
#include <geolib/HeightMap.h>
#include <geolib/Shape.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/highgui/highgui.hpp>

#include <geolib/Importer.h>

#include "xml_shape_parser.h"

// Heightmap generation
#include "polypartition/polypartition.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <geolib/CompositeShape.h>

namespace ed
{
namespace models
{

// ----------------------------------------------------------------------------------------------------

void findContours(const cv::Mat& image, const geo::Vec2i& p, std::vector<geo::Vec2i>& points)
{
    static int dx[4] = {1,  0, -1,  0 };
    static int dy[4] = {0,  1,  0, -1 };

    unsigned char v = image.at<unsigned char>(p.y, p.x);

    int d_current = 0;
    int x2 = p.x;
    int y2 = p.y;

    points.push_back(p);

    int n_uninterrupted = 0;
    int segment_length = 0;
    geo::Vec2i p_uninterrupted = p;

    while (true)
    {
        bool found = false;
        int d = (d_current + 3) % 4; // check going left first

        for(int i = 0; i < 4; ++i)
        {
            if (image.at<unsigned char>(y2 + dy[d], x2 + dx[d]) == v)
            {
                found = true;
                break;
            }

            d = (d + 1) % 4;
        }

        if (!found)
            return;

        if (d_current != d)
        {
            if (n_uninterrupted >= 3)
            {
                if (p_uninterrupted.x != points.back().x && p_uninterrupted.y != points.back().y)
                    points.push_back(p_uninterrupted);

                points.push_back(geo::Vec2i(x2, y2));
                segment_length = 0;
            }
            else if (segment_length > 10)
            {
                points.push_back(geo::Vec2i(x2, y2));
                segment_length = 0;
            }

            p_uninterrupted = geo::Vec2i(x2, y2);
            n_uninterrupted = 0;
        }
        else
        {
            ++n_uninterrupted;
        }

        ++segment_length;

        x2 = x2 + dx[d];
        y2 = y2 + dy[d];

        if (x2 == p.x && y2 == p.y)
            return;

        d_current = d;
    }
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr getHeightMapShape(const tue::filesystem::Path& path, tue::config::Reader cfg)
{
    double resolution, origin_x, origin_y, origin_z, blockheight;
    if (!(cfg.value("origin_x", origin_x) &&
            cfg.value("origin_y", origin_y) &&
            cfg.value("origin_z", origin_z) &&
            cfg.value("resolution", resolution) &&
            cfg.value("blockheight", blockheight)))
    {
        std::cout << "ed::models::getHeightMapShape() : ERROR while loading heightmap parameters at '" << path.string() << "'. Required shape parameters: resolution, origin_x, origin_y, origin_z, blockheight" << std::endl;
        return geo::ShapePtr();
    }

    cv::Mat image = cv::imread(path.string(), CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if (!image.data)
    {
        std::cout << "ed::models::getHeightMapShape() : ERROR loading heightmap at '" << path.string() << "'. Image constains invalid data." << std::endl;
        return geo::ShapePtr();;
    }

    cv::Mat vertex_index_map(image.rows, image.cols, CV_32SC1, -1);

    boost::shared_ptr<geo::CompositeShape> shape(new geo::CompositeShape);

    for(int y = 0; y < image.rows; ++y)
    {
        for(int x = 0; x < image.cols; ++x)
        {
            unsigned char v = image.at<unsigned char>(y, x);

            if (v != 255)
            {
                std::vector<geo::Vec2i> points;
                findContours(image, geo::Vec2i(x, y), points);

                int num_points = points.size();

                if (num_points > 2)
                {
                    TPPLPoly poly;
                    poly.Init(num_points);

                    geo::Mesh mesh;

                    double min_z = origin_z;
                    double max_z = origin_z + blockheight - ((double)v / 255 * blockheight);

                    for(unsigned int i = 0; i < num_points; ++i)
                    {
                        poly[i].x = points[i].x;
                        poly[i].y = points[i].y;

                        // Convert to world coordinates
                        double wx = points[i].x * resolution + origin_x;
                        double wy = (image.rows - points[i].y - 1) * resolution + origin_y;

                        vertex_index_map.at<int>(points[i].y, points[i].x) = 2 * i;

                        mesh.addPoint(geo::Vector3(wx, wy, min_z));
                        mesh.addPoint(geo::Vector3(wx, wy, max_z));
                    }

                    // Calculate side triangles
                    for(int i = 0; i < num_points; ++i)
                    {
                        int j = (i + 1) % num_points;
                        mesh.addTriangle(i * 2, j * 2, i * 2 + 1);
                        mesh.addTriangle(i * 2 + 1, j * 2, j * 2 + 1);
                    }

                    TPPLPartition pp;
                    std::list<TPPLPoly> testpolys, result;
                    testpolys.push_back(poly);

                    if (!pp.Triangulate_EC(&poly, &result))
                    {
                        std::cout << "ed::models::getHeightMapShape() : ERROR: could not triangulate polygon." << std::endl;
                        return shape;
                    }

                    for(std::list<TPPLPoly>::iterator it = result.begin(); it != result.end(); ++it)
                    {
                        TPPLPoly& cp = *it;

                        int i1 = vertex_index_map.at<int>(cp[0].y, cp[0].x) + 1;
                        int i2 = vertex_index_map.at<int>(cp[1].y, cp[1].x) + 1;
                        int i3 = vertex_index_map.at<int>(cp[2].y, cp[2].x) + 1;

                        mesh.addTriangle(i1, i2, i3);
                    }

                    geo::Shape sub_shape;
                    sub_shape.setMesh(mesh);

                    shape->addShape(sub_shape, geo::Pose3D::identity());

                    cv::floodFill(image, cv::Point(x, y), 255);
                }
            }
        }
    }

    return shape;
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr loadShape(const std::string& model_path, tue::config::Reader cfg, std::map<std::string, geo::ShapePtr>& shape_cache)
{
    geo::ShapePtr shape;

    std::string path;
    if (cfg.value("path", path))
    {
        tue::filesystem::Path shape_path(model_path + "/" + path);

        // Check cache first
        std::map<std::string, geo::ShapePtr>::const_iterator it = shape_cache.find(shape_path.string());
        if (it != shape_cache.end())
            return it->second;

        if (shape_path.exists())
        {
            std::string xt = shape_path.extension();
            if (xt == ".pgm")
            {
                shape = getHeightMapShape(shape_path, cfg);
            }
            else if (xt == ".geo")
            {
                geo::serialization::registerDeserializer<geo::Shape>();
                shape = geo::serialization::fromFile(shape_path.string());
            }
            else if (xt == ".3ds")
            {
                shape = geo::Importer::readMeshFile(shape_path.string());
            }
            else if (xt == ".xml")
            {
                std::string error;
                shape = parseXMLShape(shape_path.string(), error);
            }

            if (!shape)
                std::cout << "ed::models::loadShape() : ERROR while loading shape at " << shape_path.string() << std::endl;
            else
                // Add to cache
                shape_cache[shape_path.string()] = shape;
        }
        else
        {
            std::cout << "ed::models::loadShape() : ERROR while loading shape of at " << shape_path.string() << " ; file does not exist" << std::endl;
        }
    }
    else
    {
        std::cout << "ed::models::loadShape() : ERROR while loading shape, no path specified in model.yaml" << std::endl;
    }

    return shape;
}

}
}
