#include "shape_loader.h"

#include "xml_shape_parser.h"

#include <tue/filesystem/path.h>

#include <geolib/serialization.h>
#include <geolib/HeightMap.h>
#include <geolib/Shape.h>
#include <geolib/CompositeShape.h>
#include <geolib/Importer.h>

// Heightmap generation
#include "polypartition/polypartition.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ed
{
namespace models
{

// ----------------------------------------------------------------------------------------------------

void findContours(const cv::Mat& image, const geo::Vec2i& p_start, int d_start, std::vector<geo::Vec2i>& points,
                  std::vector<geo::Vec2i>& line_starts, cv::Mat& contour_map)
{
    static int dx[4] = {1,  0, -1,  0 };
    static int dy[4] = {0,  1,  0, -1 };

    unsigned char v = image.at<unsigned char>(p_start.y, p_start.x);

    int d = d_start; // Current direction
    geo::Vec2i p = p_start;

    while (true)
    {
        if (d == 3) // up
            line_starts.push_back(p);

        if (d == 3)
            contour_map.at<unsigned char>(p.y, p.x - 1) = 1;
        else if (d == 1)
            contour_map.at<unsigned char>(p.y, p.x) = 1;

        if (image.at<unsigned char>(p.y + dy[(d + 3) % 4], p.x + dx[(d + 3) % 4]) == v) // left
        {
            switch (d)
            {
                case 0: points.push_back(p - geo::Vec2i(1, 1)); break;
                case 1: points.push_back(p - geo::Vec2i(0, 1)); break;
                case 2: points.push_back(p); break;
                case 3: points.push_back(p - geo::Vec2i(1, 0)); break;
            }

            d = (d + 3) % 4;
            p.x += dx[d];
            p.y += dy[d];
        }
        else if (image.at<unsigned char>(p.y + dy[d], p.x + dx[d]) == v) // straight ahead
        {
            p.x += dx[d];
            p.y += dy[d];
        }
        else // right
        {
            switch (d)
            {
                case 0: points.push_back(p - geo::Vec2i(0, 1)); break;
                case 1: points.push_back(p); break;
                case 2: points.push_back(p - geo::Vec2i(1, 0)); break;
                case 3: points.push_back(p - geo::Vec2i(1, 1)); break;

            }

            d = (d + 1) % 4;
        }

        if (p.x == p_start.x && p.y == p_start.y && d == d_start)
            return;
    }
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr getHeightMapShape(const std::string& image_filename, const geo::Vec3& pos, double blockheight, double resolution, std::stringstream& error)
{
    cv::Mat image_orig = cv::imread(image_filename, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if (!image_orig.data)
    {
        error << "[ED::MODELS::LOADSHAPE] Error while loading heightmap '" << image_filename << "'. Image could not be loaded." << std::endl;
        return geo::ShapePtr();
    }

    // Add borders
    cv::Mat image(image_orig.rows + 2, image_orig.cols + 2, CV_8UC1, cv::Scalar(255));
    image_orig.copyTo(image(cv::Rect(cv::Point(1, 1), cv::Size(image_orig.cols, image_orig.rows))));

    cv::Mat vertex_index_map(image.rows, image.cols, CV_32SC1, -1);
    cv::Mat contour_map(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    boost::shared_ptr<geo::CompositeShape> shape(new geo::CompositeShape);

    for(int y = 0; y < image.rows; ++y)
    {
        for(int x = 0; x < image.cols; ++x)
        {
            unsigned char v = image.at<unsigned char>(y, x);

            if (v < 255)
            {
                std::vector<geo::Vec2i> points, line_starts;
                findContours(image, geo::Vec2i(x, y), 0, points, line_starts, contour_map);

                unsigned int num_points = points.size();

                if (num_points > 2)
                {
                    geo::Mesh mesh;

                    double min_z = pos.z;
                    double max_z = pos.z + (double)(255 - v) / 255 * blockheight;

                    std::list<TPPLPoly> testpolys;

                    TPPLPoly poly;
                    poly.Init(num_points);

                    for(unsigned int i = 0; i < num_points; ++i)
                    {
                        poly[i].x = points[i].x;
                        poly[i].y = points[i].y;

                        // Convert to world coordinates
                        double wx = points[i].x * resolution + pos.x;
                        double wy = (image.rows - points[i].y - 2) * resolution + pos.y;

                        vertex_index_map.at<int>(points[i].y, points[i].x) = mesh.addPoint(geo::Vector3(wx, wy, min_z));
                        mesh.addPoint(geo::Vector3(wx, wy, max_z));
                    }

                    testpolys.push_back(poly);

                    // Calculate side triangles
                    for(unsigned int i = 0; i < num_points; ++i)
                    {
                        int j = (i + 1) % num_points;
                        mesh.addTriangle(i * 2, i * 2 + 1, j * 2);
                        mesh.addTriangle(i * 2 + 1, j * 2 + 1, j * 2);
                    }

                    for(unsigned int i = 0; i < line_starts.size(); ++i)
                    {
                        int x2 = line_starts[i].x;
                        int y2 = line_starts[i].y;

                        while(image.at<unsigned char>(y2, x2) == v)
                            ++x2;

                        if (contour_map.at<unsigned char>(y2, x2 - 1) == 0)
                        {
                            // found a hole, so find the contours of this hole
                            std::vector<geo::Vec2i> hole_points;
                            findContours(image, geo::Vec2i(x2 - 1, y2 + 1), 1, hole_points, line_starts, contour_map);

                            if (hole_points.size() > 2)
                            {
                                TPPLPoly poly_hole;
                                poly_hole.Init(hole_points.size());
                                poly_hole.SetHole(true);

                                for(unsigned int j = 0; j < hole_points.size(); ++j)
                                {
                                    poly_hole[j].x = hole_points[j].x;
                                    poly_hole[j].y = hole_points[j].y;

                                    // Convert to world coordinates
                                    double wx = hole_points[j].x * resolution + pos.x;
                                    double wy = (image.rows - hole_points[j].y - 2) * resolution + pos.y;

                                    vertex_index_map.at<int>(hole_points[j].y, hole_points[j].x) = mesh.addPoint(geo::Vector3(wx, wy, min_z));
                                    mesh.addPoint(geo::Vector3(wx, wy, max_z));
                                }
                                testpolys.push_back(poly_hole);

                                // Calculate side triangles
                                for(unsigned int j = 0; j < hole_points.size(); ++j)
                                {
                                    const geo::Vec2i& hp1 = hole_points[j];
                                    const geo::Vec2i& hp2 = hole_points[(j + 1) % hole_points.size()];

                                    int i1 = vertex_index_map.at<int>(hp1.y, hp1.x);
                                    int i2 = vertex_index_map.at<int>(hp2.y, hp2.x);

                                    mesh.addTriangle(i1, i1 + 1, i2);
                                    mesh.addTriangle(i2, i1 + 1, i2 + 1);
                                }
                            }
                        }
                    }

                    TPPLPartition pp;
                    std::list<TPPLPoly> result;

                    if (!pp.Triangulate_EC(&testpolys, &result))
                    {
                        error << "[ED::MODELS::LOADSHAPE] Error while creating heightmap: could not triangulate polygon." << std::endl;
                        return shape;
                    }

                    for(std::list<TPPLPoly>::iterator it = result.begin(); it != result.end(); ++it)
                    {
                        TPPLPoly& cp = *it;

                        int i1 = vertex_index_map.at<int>(cp[0].y, cp[0].x) + 1;
                        int i2 = vertex_index_map.at<int>(cp[1].y, cp[1].x) + 1;
                        int i3 = vertex_index_map.at<int>(cp[2].y, cp[2].x) + 1;

                        mesh.addTriangle(i1, i3, i2);
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

geo::ShapePtr getHeightMapShape(const tue::filesystem::Path& path, tue::config::Reader cfg, std::stringstream& error)
{
    double resolution, origin_x, origin_y, origin_z, blockheight;
    if (!(cfg.value("origin_x", origin_x) &&
            cfg.value("origin_y", origin_y) &&
            cfg.value("origin_z", origin_z) &&
            cfg.value("resolution", resolution) &&
            cfg.value("blockheight", blockheight)))
    {
        error << "[ED::MODELS::LOADSHAPE] Error while loading heightmap parameters at '" << path.string()
              << "'. Required shape parameters: resolution, origin_x, origin_y, origin_z, blockheight" << std::endl;
        return geo::ShapePtr();
    }

    return getHeightMapShape(path.string(), geo::Vec3(origin_x, origin_y, origin_z), blockheight, resolution, error);
}

// ----------------------------------------------------------------------------------------------------

void createPolygon(geo::Shape& shape, const std::vector<geo::Vec2>& points, double height, bool create_bottom)
{
    TPPLPoly poly;
    poly.Init(points.size());

    double min_z = -height / 2;
    double max_z =  height / 2;

    geo::Mesh mesh;

    for(unsigned int i = 0; i < points.size(); ++i)
    {
        poly[i].x = points[i].x;
        poly[i].y = points[i].y;

        mesh.addPoint(geo::Vector3(points[i].x, points[i].y, min_z));
        mesh.addPoint(geo::Vector3(points[i].x, points[i].y, max_z));
    }

    // Add side triangles
    for(unsigned int i = 0; i < points.size(); ++i)
    {
        int j = (i + 1) % points.size();
        mesh.addTriangle(i * 2, j * 2, i * 2 + 1);
        mesh.addTriangle(i * 2 + 1, j * 2, j * 2 + 1);
    }

    std::list<TPPLPoly> polys;
    polys.push_back(poly);

    TPPLPartition pp;
    std::list<TPPLPoly> result;

    if (!pp.Triangulate_EC(&polys, &result))
    {
        std::cout << "TRIANGULATION FAILED" << std::endl;
        return;
    }

    for(std::list<TPPLPoly>::iterator it = result.begin(); it != result.end(); ++it)
    {
        TPPLPoly& cp = *it;

        int i1 = mesh.addPoint(cp[0].x, cp[0].y, max_z);
        int i2 = mesh.addPoint(cp[1].x, cp[1].y, max_z);
        int i3 = mesh.addPoint(cp[2].x, cp[2].y, max_z);
        mesh.addTriangle(i1, i2, i3);

        if (create_bottom)
        {
            int i1 = mesh.addPoint(cp[0].x, cp[0].y, min_z);
            int i2 = mesh.addPoint(cp[1].x, cp[1].y, min_z);
            int i3 = mesh.addPoint(cp[2].x, cp[2].y, min_z);
            mesh.addTriangle(i1, i3, i2);
        }
    }

    mesh.filterOverlappingVertices();

    shape.setMesh(mesh);
}

// ----------------------------------------------------------------------------------------------------

void createCylinder(geo::Shape& shape, double radius, double height, int num_corners)
{
    geo::Mesh mesh;

    // Calculate vertices
    for(int i = 0; i < num_corners; ++i)
    {
        double a = 6.283 * i / num_corners;
        double x = sin(a) * radius;
        double y = cos(a) * radius;

        mesh.addPoint(x, y, -height / 2);
        mesh.addPoint(x, y,  height / 2);
    }

    // Calculate top and bottom triangles
    for(int i = 1; i < num_corners - 1; ++i)
    {
        int i2 = 2 * i;

        // bottom
        mesh.addTriangle(0, i2, i2 + 2);

        // top
        mesh.addTriangle(1, i2 + 3, i2 + 1);
    }

    // Calculate side triangles
    for(int i = 0; i < num_corners; ++i)
    {
        int j = (i + 1) % num_corners;
        mesh.addTriangle(i * 2, i * 2 + 1, j * 2);
        mesh.addTriangle(i * 2 + 1, j * 2 + 1, j * 2);
    }

    shape.setMesh(mesh);
}

// ----------------------------------------------------------------------------------------------------

void readVec3(tue::config::Reader& cfg, geo::Vec3& v)
{
    cfg.value("x", v.x);
    cfg.value("y", v.y);
    cfg.value("z", v.z);
}

// ----------------------------------------------------------------------------------------------------

void readPose(tue::config::Reader& cfg, geo::Pose3D& pose)
{
    cfg.value("x", pose.t.x, tue::config::OPTIONAL);
    cfg.value("y", pose.t.y, tue::config::OPTIONAL);
    cfg.value("z", pose.t.z, tue::config::OPTIONAL);

    double roll = 0, pitch = 0, yaw = 0;
    cfg.value("X", roll,  tue::config::OPTIONAL);
    cfg.value("Y", pitch, tue::config::OPTIONAL);
    cfg.value("Z", yaw,   tue::config::OPTIONAL);
    cfg.value("roll",  roll,  tue::config::OPTIONAL);
    cfg.value("pitch", pitch, tue::config::OPTIONAL);
    cfg.value("yaw",   yaw,   tue::config::OPTIONAL);

    // Set rotation
    pose.R.setRPY(roll, pitch, yaw);
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr loadShape(const std::string& model_path, tue::config::Reader cfg,
                        std::map<std::string, geo::ShapePtr>& shape_cache, std::stringstream& error)
{
    geo::ShapePtr shape;
    geo::Pose3D pose = geo::Pose3D::identity();

    std::string path;
    if (cfg.value("path", path))
    {
        if (path.empty())
        {
            error << "[ED::MODELS::LOADSHAPE] Error while loading shape: provided path is empty.";
            return shape;
        }

        tue::filesystem::Path shape_path;

        if (model_path.empty() || path[0] == '/')
            shape_path = path;
        else
            shape_path = model_path + "/" + path;

        // Check cache first
        std::map<std::string, geo::ShapePtr>::const_iterator it = shape_cache.find(shape_path.string());
        if (it != shape_cache.end())
            return it->second;

        if (shape_path.exists())
        {
            std::string xt = shape_path.extension();
            if (xt == ".pgm")
            {
                shape = getHeightMapShape(shape_path, cfg, error);
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
                error << "[ED::MODELS::LOADSHAPE] Error while loading shape at " << shape_path.string() << std::endl;
            else
                // Add to cache
                shape_cache[shape_path.string()] = shape;
        }
        else
        {
            error << "[ED::MODELS::LOADSHAPE] Error while loading shape at " << shape_path.string() << " ; file does not exist" << std::endl;
        }
    }
    else if (cfg.readGroup("box"))
    {
        geo::Vec3 min, max;
        if (cfg.readGroup("min"))
        {
            readVec3(cfg, min);
            cfg.endGroup();

            if (cfg.readGroup("max"))
            {
                readVec3(cfg, max);
                shape.reset(new geo::Box(min, max));
                cfg.endGroup();
            }
            else
            {
                error << "[ED::MODELS::LOADSHAPE] Error while loading shape: box must contain 'min' and 'max' (only 'min' specified)";
            }
        }
        else if (cfg.readGroup("size"))
        {
            geo::Vec3 size;
            readVec3(cfg, size);
            shape.reset(new geo::Box(-0.5 * size, 0.5 * size));
            cfg.endGroup();
        }
        else
        {
            error << "[ED::MODELS::LOADSHAPE] Error while loading shape: box must contain 'min' and 'max' or 'size'.";
        }

        if (cfg.readGroup("pose"))
        {
            readPose(cfg, pose);
            cfg.endGroup();
        }
    }
    else if (cfg.readGroup("cylinder"))
    {
        int num_points = 12;
        cfg.value("num_points", num_points, tue::config::OPTIONAL);

        double radius, height;
        if (cfg.value("radius", radius) && cfg.value("height", height))
        {
            shape.reset(new geo::Shape);
            createCylinder(*shape, radius, height, num_points);
        }

        cfg.endGroup();
    }
    else if (cfg.readGroup("polygon"))
    {
        std::vector<geo::Vec2> points;
        if (cfg.readArray("points", tue::config::REQUIRED))
        {
            while(cfg.nextArrayItem())
            {
                points.push_back(geo::Vec2());
                geo::Vec2& p = points.back();
                cfg.value("x", p.x);
                cfg.value("y", p.y);
            }
            cfg.endArray();
        }

        double height;
        if (cfg.value("height", height))
        {
            shape.reset(new geo::Shape);
            createPolygon(*shape, points, height, true);
        }

        cfg.endGroup();
    }
    else if (cfg.readArray("compound") || cfg.readArray("group"))
    {
        boost::shared_ptr<geo::CompositeShape> composite(new geo::CompositeShape);
        while(cfg.nextArrayItem())
        {
            std::map<std::string, geo::ShapePtr> dummy_shape_cache;
            geo::ShapePtr sub_shape = loadShape(model_path, cfg, dummy_shape_cache, error);
            composite->addShape(*sub_shape, geo::Pose3D::identity());
        }
        cfg.endArray();

        shape = composite;
    }
    else if (cfg.readArray("heightmap"))
    {
        std::string image_filename;
        double height, resolution;

        if (cfg.value("image", image_filename) && !image_filename.empty()
                && cfg.value("resolution", resolution)
                && cfg.value("height", height))
        {
            std::string image_filename_full = image_filename;
//            if (image_filename[0] == '/')
//                image_filename_full = image_filename;
//            else
//                image_filename_full = model_path + "/" + image_filename;

            shape = getHeightMapShape(image_filename_full, geo::Vec3(0, 0, 0), height, resolution, error);

            if (cfg.readGroup("pose"))
            {
                readPose(cfg, pose);
                cfg.endGroup();
            }
        }
        else
        {
            error << "[ED::MODELS::LOADSHAPE] Error while loading shape: heightmap must contain 'image', 'resolution' and 'height'." << std::endl;
        }
    }
    else
    {
        error << "[ED::MODELS::LOADSHAPE] Error while loading shape: must contain one of the following 'path', 'heightmap', 'box', 'compound'." << std::endl;
    }

    if (cfg.readGroup("pose"))
    {
        readPose(cfg, pose);
        cfg.endGroup();
    }

    if (shape)
    {
        // Transform shape according to pose
        geo::ShapePtr shape_tr(new geo::Shape);
        shape_tr->setMesh(shape->getMesh().getTransformed(pose));
        shape = shape_tr;
    }

    return shape;
}

}
}
