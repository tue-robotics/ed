#ifndef ED_MODELS_SHAPE_LOADER_H_
#define ED_MODELS_SHAPE_LOADER_H_

#include <geolib/datatypes.h>
#include <geolib/Mesh.h>
#include <geolib/Shape.h>

#include <cmath>
#include <map>

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
void createCylinder(geo::Shape& shape, double radius, double height, int num_corners = 12);

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
uint getMiddlePoint(geo::Mesh& mesh, uint i1, uint i2, std::map<unsigned long, uint> cache, double radius);

/**
 * @brief createSphere Create a shape of sphere
 * @param shape Shape object to be filled
 * @param radius radius of the sphere
 * @param recursion_level number of recursions to smooth the mesh, but rapidly increases the mesh.
 */
void createSphere(geo::Shape& shape, double radius, uint recursion_level = 2);

} // end namespace models

} // end namespace ed

#endif
