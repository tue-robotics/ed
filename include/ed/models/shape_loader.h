#ifndef ED_SHAPE_LOADER_H_
#define ED_SHAPE_LOADER_H_

#include <geolib/datatypes.h>
#include <geolib/Mesh.h>
#include <geolib/Shape.h>

#include <cmath>

namespace ed
{

namespace models
{


/**
 * @brief createCylinder create a mesh from radius and height
 * @param shape filled mesh
 * @param radius radius of the cylinder
 * @param height height of the cylinder
 * @param num_corners divided the circumference in N points and N+1 lines
 */
void createCylinder(geo::Shape& shape, double radius, double height, int num_corners = 12)
{
    geo::Mesh mesh;

    // Calculate vertices
    for(int i = 0; i < num_corners; ++i)
    {
        double a = 2 * M_PI * i / num_corners;
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
/**
 * @brief getMiddlePoint Gets the middle point of two points in a mesh of a sphere. Uses a cache to not create double points.
 * The new point is placed on the radius of the sphere.
 * @param mesh Mesh of the sphere
 * @param i1 index of first point
 * @param i2 index of second point
 * @param cache cache of the middle points
 * @param radius radius of teh sphere
 * @return index of the inserted point
 */
int getMiddlePoint(geo::Mesh& mesh, int i1, int i2, std::map<long, int> cache, double radius)
{
       // first check if we have it already
       bool firstIsSmaller = i1 < i2;
       long smallerIndex = firstIsSmaller ? i1 : i2;
       long greaterIndex = firstIsSmaller ? i2 : i1;
       long key = (smallerIndex << 32) + greaterIndex;

       std::map<long, int>::const_iterator it = cache.find(key);
       if (it != cache.end())
           return it->second;

       // not in cache, calculate it
       const std::vector<geo::Vec3>& points = mesh.getPoints();
       geo::Vec3 p1 = points[i1];
       geo::Vec3 p2 = points[i2];
       geo::Vec3 p3((p1+p2)/2);
       p3 = p3.normalized() * radius;

       // add vertex makes sure point is on unit sphere
       int i3 = mesh.addPoint(p3);

       // store it, return index
       cache.insert(std::pair<long, int>(key, i3));
       return i3;
}
/**
 * @brief createSphere Create a shape of sphere
 * @param shape Shape object to be filled
 * @param radius radius of the sphere
 * @param recursion_level number of recursions to smooth the mesh, but rapidly increases the mesh.
 */
void createSphere(geo::Shape& shape, double radius, int recursion_level = 2)
{
    geo::Mesh mesh;

    // create 12 vertices of a icosahedron
    double t = (1.0 + sqrt(5.0)) / 2.0;

    mesh.addPoint(geo::Vec3(-1,  t,  0).normalized()*radius);
    mesh.addPoint(geo::Vec3( 1,  t,  0).normalized()*radius);
    mesh.addPoint(geo::Vec3(-1, -t,  0).normalized()*radius);
    mesh.addPoint(geo::Vec3( 1, -t,  0).normalized()*radius);

    mesh.addPoint(geo::Vec3( 0, -1,  t).normalized()*radius);
    mesh.addPoint(geo::Vec3( 0,  1,  t).normalized()*radius);
    mesh.addPoint(geo::Vec3( 0, -1, -t).normalized()*radius);
    mesh.addPoint(geo::Vec3( 0,  1, -t).normalized()*radius);

    mesh.addPoint(geo::Vec3( t,  0, -1).normalized()*radius);
    mesh.addPoint(geo::Vec3( t,  0,  1).normalized()*radius);
    mesh.addPoint(geo::Vec3(-t,  0, -1).normalized()*radius);
    mesh.addPoint(geo::Vec3(-t,  0,  1).normalized()*radius);

    // create 20 triangles of the icosahedron
    // 5 faces around point 0
    mesh.addTriangle(0, 11, 5);
    mesh.addTriangle(0, 5, 1);
    mesh.addTriangle(0, 1, 7);
    mesh.addTriangle(0, 7, 10);
    mesh.addTriangle(0, 10, 11);

    // 5 adjacent faces
    mesh.addTriangle(1, 5, 9);
    mesh.addTriangle(5, 11, 4);
    mesh.addTriangle(11, 10, 2);
    mesh.addTriangle(10, 7, 6);
    mesh.addTriangle(7, 1, 8);

    // 5 faces around point 3
    mesh.addTriangle(3, 9, 4);
    mesh.addTriangle(3, 4, 2);
    mesh.addTriangle(3, 2, 6);
    mesh.addTriangle(3, 6, 8);
    mesh.addTriangle(3, 8, 9);

    // 5 adjacent faces
    mesh.addTriangle(4, 9, 5);
    mesh.addTriangle(2, 4, 11);
    mesh.addTriangle(6, 2, 10);
    mesh.addTriangle(8, 6, 7);
    mesh.addTriangle(9, 8, 1);

    for (int i = 0; i < recursion_level; i++)
    {
        geo::Mesh mesh2;
        std::map<long, int> cache;

        const std::vector<geo::Vec3>& points = mesh.getPoints();
        for (std::vector<geo::Vec3>::const_iterator it = points.begin(); it != points.end(); ++it)
            mesh2.addPoint(*it);

        const std::vector<geo::TriangleI>& triangleIs = mesh.getTriangleIs();
        for (std::vector<geo::TriangleI>::const_iterator it = triangleIs.begin(); it != triangleIs.end(); ++it)
        {
            // replace triangle by 4 triangles
            int a = getMiddlePoint(mesh2, it->i1_, it->i2_, cache, radius);
            int b = getMiddlePoint(mesh2, it->i2_, it->i3_, cache, radius);
            int c = getMiddlePoint(mesh2, it->i3_, it->i1_, cache, radius);

            mesh2.addTriangle(it->i1_, a, c);
            mesh2.addTriangle(it->i2_, b, a);
            mesh2.addTriangle(it->i3_, c, b);
            mesh2.addTriangle(a, b, c);
        }
        mesh = mesh2;
    }
    shape.setMesh(mesh);
}

} // end namespace models

} // end namespace ed

#endif
