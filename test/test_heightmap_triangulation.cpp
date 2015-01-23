#include "polypartition/polypartition.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geolib/Mesh.h>
#include <geolib/Shape.h>
#include <geolib/serialization.h>

#include <iostream>

// ----------------------------------------------------------------------------------------------------

void findContours(const cv::Mat& image, const geo::Vec2i& p, int d_start, std::vector<geo::Vec2i>& points,
                  std::vector<geo::Vec2i>& line_starts, cv::Mat& contour_map, bool add_first)
{
    static int dx[4] = {1,  0, -1,  0 };
    static int dy[4] = {0,  1,  0, -1 };

    unsigned char v = image.at<unsigned char>(p.y, p.x);

    int d_current = d_start; // Current direction
    int x2 = p.x;
    int y2 = p.y;

    int line_piece_min = 1e9; // minimum line piece length of current line
    int line_piece_max = 0; // maximum line piece length of current line

    int d_main = d_current; // The main direction in which we're heading. If we follow a line
                            // that gradually changes to the side (1-cell side steps), this direction
                            // denotes the principle axis of the line

    if (add_first)
        points.push_back(p - geo::Vec2i(1, 1));

    int n_uninterrupted = 1;
    geo::Vec2i p_corner = p;

    while (true)
    {
        bool found = false;
        int d = (d_current + 3) % 4; // check going left first

        for(int i = -1; i < 3; ++i)
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

        geo::Vec2i p_current(x2, y2);

        if ((d + 2) % 4 == d_current)
        {
            // 180 degree turn

            if (x2 == p.x && y2 == p.y) // Edge case: if we returned to the start point and
                                        // this is a 180 degree angle, return without adding it
                return;


            geo::Vec2i q = p_current;
            if (d == 0 || d_current == 0) // right
                --q.y;
            if (d == 3 || d_current == 3) // up
                --q.x;

            points.push_back(q);
            d_main = d;
            line_piece_min = 1e9;
            line_piece_max = 0;

        }
        else if (d_current != d_main)
        {
            // Not moving in main direction (side step)

            if (d != d_main)
            {
                // We are not moving back in the main direction
                // Add the corner to the list and make this our main direction

                points.push_back(p_corner);
                d_main = d_current;
                line_piece_min = 1e9;
                line_piece_max = 0;
            }
        }
        else
        {
            // Moving in main direction (no side step)

            if (d_current != d)
            {
                // Turning 90 degrees

                // Check if the length of the last line piece is OK w.r.t. the other pieces in this line. If it differs to much,
                // (i.e., the contour has taken a different angle), add the last corner to the list. This way, we introduce a
                // bend in the contour
                if (line_piece_max > 0 && (n_uninterrupted < line_piece_max - 2 || n_uninterrupted > line_piece_min + 2))
                {
                    // Line is broken, add the corner as bend
                    points.push_back(p_corner);

                    line_piece_min = 1e9;
                    line_piece_max = 0;
                }

                // Update the line piece lenth boundaries with the current found piece
                line_piece_min = std::min(line_piece_min, n_uninterrupted);
                line_piece_max = std::max(line_piece_max, n_uninterrupted);
            }
        }

        if (d_current != d)
        {
            geo::Vec2i q = p_current;
            if (d == 0 || d_current == 0) // right
                --q.y;
            if (d == 3 || d_current == 3) // up
                --q.x;

            p_corner = q;
            n_uninterrupted = 0;
        }

        if ((d_current == 3 && d != 2) || (d == 3 && d != 0)) // up
            line_starts.push_back(p_current);

        contour_map.at<unsigned char>(p_current.y, p_current.x) = 1;

        ++n_uninterrupted;

        if (points.size() > 1 && x2 == p.x && y2 == p.y)
            return;

        x2 = x2 + dx[d];
        y2 = y2 + dy[d];

        d_current = d;
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc <= 1)
        return 0;

    std::string image_filename = argv[1];
    std::cout << "Loading " << image_filename << std::endl;

    cv::Mat image = cv::imread(image_filename, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    cv::Mat viz(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for(int y = 0; y < image.rows; ++y)
    {
        for(int x = 0; x < image.cols; ++x)
        {
            unsigned char v = image.at<unsigned char>(y, x);
            viz.at<cv::Vec3b>(y, x) = cv::Vec3b(v, v, v);
        }
    }

    if (image.data)
    {
        std::cout << "Successfully loaded" << std::endl;

        cv::Mat vertex_index_map(image.rows, image.cols, CV_32SC1, -1);
        cv::Mat contour_map(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

        for(int y = 0; y < image.rows; ++y)
        {
            for(int x = 0; x < image.cols; ++x)
            {
                unsigned char v = image.at<unsigned char>(y, x);

                if (v != 255)
                {
                    std::vector<geo::Vec2i> points, line_starts;
                    findContours(image, geo::Vec2i(x, y), 0, points, line_starts, contour_map, true);

                    int num_points = points.size();

                    std::cout << std::endl << "Total polygon size: " << points.size() << std::endl;

                    if (num_points > 2)
                    {

                        for(unsigned int i = 0; i < num_points; ++i)
                        {
                            int k = (i + 1) % num_points;
//                            cv::line(viz, cv::Point(points[i].x, points[i].y),
//                                     cv::Point(points[k].x, points[k].y), cv::Scalar(0, 0, 255));
                        }

                        geo::Mesh mesh;

                        double min_z = 0;
                        double max_z = 0;

                        std::list<TPPLPoly> testpolys;

                        TPPLPoly poly;
                        poly.Init(num_points);

                        for(unsigned int i = 0; i < num_points; ++i)
                        {
                            poly[i].x = points[i].x;
                            poly[i].y = points[i].y;

                            // Convert to world coordinates
                            double wx = points[i].x * 0.025;
                            double wy = points[i].y * 0.025;

                            vertex_index_map.at<int>(points[i].y, points[i].x) = mesh.addPoint(geo::Vector3(wx, wy, min_z));
                            mesh.addPoint(geo::Vector3(wx, wy, max_z));
                        }

                        testpolys.push_back(poly);

                        // Calculate side triangles
                        for(int i = 0; i < num_points; ++i)
                        {
                            int j = (i + 1) % num_points;
                            mesh.addTriangle(i * 2, j * 2, i * 2 + 1);
                            mesh.addTriangle(i * 2 + 1, j * 2, j * 2 + 1);
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
                                findContours(image, geo::Vec2i(x2 - 1, y2 + 1), 1, hole_points, line_starts, contour_map, false);

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
                                        double wx = hole_points[j].x * 0.025;
                                        double wy = hole_points[j].y * 0.025;

                                        vertex_index_map.at<int>(hole_points[j].y, hole_points[j].x) = mesh.addPoint(geo::Vector3(wx, wy, min_z));
                                        mesh.addPoint(geo::Vector3(wx, wy, max_z));
                                    }
                                    testpolys.push_back(poly_hole);
                                }
                            }
                        }

                        TPPLPartition pp;
                        std::list<TPPLPoly> result;

                        if (!pp.Triangulate_EC(&testpolys, &result))
                        {
                            std::cout << "Error" << std::endl;
                        }
                        else
                        {
                            std::cout << "Number of triangles: " << result.size() << std::endl << std::endl;

                            for(std::list<TPPLPoly>::iterator it = result.begin(); it != result.end(); ++it)
                            {
                                TPPLPoly& cp = *it;

                                int i1 = vertex_index_map.at<int>(cp[0].y, cp[0].x) + 1;
                                int i2 = vertex_index_map.at<int>(cp[1].y, cp[1].x) + 1;
                                int i3 = vertex_index_map.at<int>(cp[2].y, cp[2].x) + 1;

                                mesh.addTriangle(i1, i2, i3);

                                // visualize
                                for(unsigned j = 0; j < cp.GetNumPoints(); ++j)
                                {
                                    int k = (j + 1) % cp.GetNumPoints();
                                    cv::line(viz, cv::Point(cp[j].x, cp[j].y), cv::Point(cp[k].x, cp[k].y), cv::Scalar(0, 0, 255));
                                }
                            }
                        }


                    }

                    cv::floodFill(image, cv::Point(x, y), 255);
                }
            }
        }        

        cv::imshow("contours", contour_map * 255);
        cv::imshow("image", viz);
        cv::waitKey();
    }
    else
    {
        std::cout << "ed::models::getHeightMapShape() : ERROR loading heightmap at '" << image_filename << "'. Image constains invalid data." << std::endl;
    }
}
