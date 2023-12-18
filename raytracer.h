#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <Eigen/Dense>
#include <optional>
#include <fstream>
#include <map>

using std::string, Eigen::Vector3d;

struct Ray;
struct Collision;

struct Sun;
struct Bulb;
struct Plane;
struct Vertex;
struct Sphere;
struct Triangle;

struct Geometry;
struct GeometryPrinter;

struct Texture;

bool operator==(const Sphere& a, const Sphere& b);
bool operator==(const Plane& a, const Plane& b);
bool operator==(const Vertex& a, const Vertex& b);
bool operator==(const Triangle& a, const Triangle& b);

/**
 * Calculates whether an object is blocking the light traveling to the intersection point.
 * @param light_source the light source
 * @param collided_object the object the primary ray collided with 
 * @param intersection the intersection point p
 * @param primary the primary ray (original ray that collided with an obj)
 * @param debug prints out whether a secondary ray collided with a new object
 * @returns whether an object blocks the light source trajectory to intersection point p
*/
bool findShadows(Geometry light_source, Geometry collided_object, Vector3d intersection, Ray primary, bool debug = false);

/**
 * Calculates a ray for every pixel in the image and saves to a vector of rays
 * @param debug prints out how the rays are stored in the vector
 * @returns a vector of rays of size w*h
*/
std::vector<Ray> computeRays(bool debug = false);

/**
 * Calculates the color of every pixel in the final scene based on Lambertian lighting,
 * then converts linear space illumination into sRGB before saving color to the pixel
 * @param debug prints out a 10x25 top-left chunk of the final scene data
 * @returns a vector of colored pixels that make up the final scene
*/
std::vector<unsigned char> calculateIlluminations(bool debug = false);

/**
 * Determines lighting on a scene object based on the light object.
 * @param light the light object
 * @param n the normal of the scene object
 * @param ray the ray
 * @param objcolor the color of the scene object
 * @returns the Lambert dot product otherwise nullopt if the light object is not a light
*/
std::optional<Vector3d> lambert(const Geometry &light, const Vector3d &n, const Ray &ray, const Vector3d &objcolor);

/**
 * Determines global illumination
 * @param intersection the intersection point of the object
 * @param obj the object
*/
Vector3d gi(const Vector3d &intersection, const Geometry &obj);

/**
 * A recursive method to calculate shininess of a sphere
 * @param obj the sphere
 * @param ray the ray
 * @param depth the depth level of recursion
 * @returns a color vector with shininess included
*/
Vector3d shiny(const Sphere &obj, const Ray &ray, unsigned depth = 0);

/**
 * Goes through every ray and checks whether it collides with any of the stored
 * objects and picks the closest collision to an object.
 * @param debug prints out whether an collision was found for every ray
 * @returns a ray-collision map that given a ray's associated (x, y) pixel as a key,
 * returns collision data for that ray as the value (no key means no collision)
*/
std::map<std::pair<int, int>, Collision> findIntersections(bool debug = false);

/**
 * Goes through the image data, processes any mode/state changes, loads
 * any objects found in the data and saves each of them in a vector of objects
 * @param file .txt file with image data
 * @param debug prints out how the objects are stored in the vector
 * @returns a vector of objects that were found in the data
*/
std::vector<Geometry> processImageData(std::ifstream& file, bool debug = false);

/**
 * The ray-sphere intersection algorithm
 * @param ray an input ray at (x, y)
 * @param sphere an input sphere object
 * @returns a pair with the intersection distance t and the point of intersection, 
 * otherwise nullopt if no intersection was found
*/
std::optional<std::pair<double, Vector3d>> raySphereIntersection(const Ray &ray, const Sphere &sphere);

/**
 * Retrieves u, v texture coordinates for a sphere (SPHERES ONLY)
 * @param p an input intersection point
 * @param sphere an input sphere object
 * @returns a pair (u, v) of texture coordinates
*/
std::pair<double, double> textureCoordinates(const Vector3d &p, const Sphere &sphere);

/**
 * The ray-plane intersection algorithm
 * @param ray an input ray at (x, y)
 * @param plane an input plane object
 * @returns a pair with the intersection distance t and the point of intersection, 
 * otherwise nullopt if no intersection was found
*/
std::optional<std::pair<double, Vector3d>> rayPlaneIntersection(const Ray &ray, const Plane &plane);

/**
 * The ray-triangle intersection algorithm (a variant of ray-plane)
 * @param ray an input ray at (x, y)
 * @param triangle an input triangle object
 * @returns a pair with the intersection distance t and the point of intersection, 
 * otherwise nullopt if no intersection was found
*/
std::optional<std::pair<double, Vector3d>> rayTriangleIntersection(const Ray &ray, const Triangle &triangle);

/**
 * Barycentric coordinate converter (aka point-in-triangle testing)
 * A point is in a triangle if all Barycentric coordinates b0, b1, b2 are in 0 <= b <= 1
 * @param p an input intersection point
 * @param triangle an input triangle object
 * @returns the Barycentric coordinates b0, b1, b2 and whether the point is in the triangle
*/
std::pair<Vector3d, bool> barycentric(const Vector3d &p, const Triangle &triangle);

/**
 * Initializes global variables for the image data like width, height, and file name
 * @param file .txt file with image data
*/
void setImageGlobals(std::ifstream& file);

/**
 * Main function that generates the image via raytracing, used in main.cpp
 * @param file .txt file with image data
 * @returns a pair of pairs, first pair is (scene data, scene name), second pair is (x, y) of scene dimensions
*/
std::pair<std::pair<std::vector<unsigned char>, string>, std::pair<int, int>> generateImage(std::ifstream& file);

/**
 * Utility function that opens the image data .txt file
 * @param argc command line argument
 * @param argv command line argument
 * @param debug prints the file details
*/
std::ifstream openFile(int argc, char** argv, bool debug = false);

#endif
