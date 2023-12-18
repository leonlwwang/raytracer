#include "raytracer.h"
#include "lodepng.h"

#include <Eigen/Dense>
#include <algorithm>
#include <optional>
#include <iostream>
#include <fstream>
#include <sstream>
#include <variant>
#include <random>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <stack>
#include <map>

using std::string, Eigen::Vector3d;

struct Ray {
    std::pair<int, int> points;
    std::pair<double, double> scalars;
    Vector3d origin;
    Vector3d direction;
    bool skip;
};

struct Collision {
    double distance;
    Vector3d intersection;
    Geometry *collided_obj;
};

struct Sun {
    Vector3d direction;
    Vector3d color;
};

struct Bulb {
    Vector3d position;
    Vector3d color;
};

struct Sphere {
    Vector3d center;
    Vector3d color;
    double radius;
    std::shared_ptr<Texture> texture;
    Vector3d shininess;
};

struct Plane {
    Vector3d normal;
    double d;
    Vector3d color;
};

struct Vertex {
    Vector3d position;
    std::pair<double, double> texcoord;
};

struct Triangle {
    std::shared_ptr<Vertex> v1, v2, v3;
    Vector3d normal;
    Vector3d e1, e2;
    Vector3d color;
    std::shared_ptr<Texture> texture;
};

struct Geometry {
    std::variant<Sphere, Sun, Plane, Triangle, Bulb> geometry;
};

struct Texture {
    std::vector<unsigned char> data;
    unsigned w, h;
    std::map<std::pair<int, int>, std::pair<double, double>> texcoord;
};

struct GeometryPrinter {
    void operator()(const Sphere& sphere) const {
        std::cout << "Sphere { Center: " << "(" << sphere.center.transpose() << ")" 
                  << ", Color: " << "(" << sphere.color.transpose() << ")"
                  << ", Radius: " << sphere.radius << " }\n";
    }

    void operator()(const Sun& sun) const {
        std::cout << "Sun { Direction: " << "(" << sun.direction.transpose() << ")" 
                  << ", Color: " << "(" << sun.color.transpose() << ")" << " }\n";
    }

    void operator()(const Plane& plane) const {
        std::cout << "Plane { Normal: " << "(" << plane.normal.transpose() << ")" 
                  << ", Offset (d): " << plane.d << ", Color: " << "(" 
                  << plane.color.transpose() << ")" << " }\n";
    }

    void operator()(const Triangle& tri) const {
        std::cout << "Triangle { Normal: " << "(" << tri.normal.transpose() << ")" 
                  << ", Color: " << "(" << tri.color.transpose() << ")" << " }\n";
    }

    void operator()(const Bulb& bulb) const {
        std::cout << "Bulb { Position: " << "(" << bulb.position.transpose() << ")" 
                  << ", Color: " << "(" << bulb.color.transpose() << ")" << " }\n";
    }
};

bool operator==(const Sphere& a, const Sphere& b) {
    return a.center.isApprox(b.center) && a.radius == b.radius && a.color.isApprox(b.color);
}

bool operator==(const Plane& a, const Plane& b) {
    return a.normal.isApprox(b.normal) && a.d == b.d;
}

bool operator==(const Vertex& a, const Vertex& b) {
    return a.position.isApprox(b.position) && a.texcoord == b.texcoord;
}

bool operator==(const Triangle& a, const Triangle& b) {
    return a.normal.isApprox(b.normal) && a.v1 == b.v1 && a.v2 == b.v2 && a.v3 == b.v3 && a.color.isApprox(b.color);
}

/* Image data globals */
int w_ = -1;                                                                /* PNG width */
int h_ = -1;                                                                /* PNG height*/
string png_ = "";                                                           /* PNG filename */

/* Raytracer globals */
std::map<std::pair<int, int>, Collision> collisions_;                       /* Ray-collision map */
std::vector<Ray> rays_;                                                     /* Rays */
std::vector<Geometry> objects_;                                             /* Objects */
Vector3d color_(1, 1, 1);                                                   /* Color */
Vector3d e_(0, 0, 0);                                                       /* Eye location */
Vector3d f_(0, 0, -1);                                                      /* Forward vector */
Vector3d r_(1, 0, 0);                                                       /* Right vector */
Vector3d u_(0, 1, 0);                                                       /* Up vector */

/* Raytracer elective globals */
std::pair<bool, double> expose_ = std::pair{false, -1};                     /* Exposure setting */
bool fisheye_ = false;                                                      /* Fisheye setting */
bool panorama_ = false;                                                     /* Panorama setting */
std::stack<Texture> textures_;                                              /* Texture setting */
Vector3d shiny_(0, 0, 0);                                                   /* Shininess setting */
unsigned bounce_ = 4;                                                       /* Bounce setting */
unsigned aa_ = 0;                                                           /* Anti-aliasing setting */
double focus_ = -1, lens_ = -1;                                             /* Depth of field setting */
unsigned gi_ = 0;                                                           /* Global illumination setting */
std::vector<Collision> aa_collisions_;                                      /* Ray-collision map for anti-aliasing */
std::map<std::pair<int, int>, double> alphas_;                              /* Alpha values for anti-aliasing */
double epsilon_ = 1e-4;                                                     /* Shadow ray epsilon */
std::vector<std::shared_ptr<Vertex>> vertices_;                             /* Vertex data for triangles */
std::pair<double, double> texcoord_ = {0.0, 0.0};                           /* Texture coordinate for vertices */

std::pair<std::pair<std::vector<unsigned char>, string>, std::pair<int, int>> generateImage(std::ifstream& file) {
    setImageGlobals(file);
    objects_ = processImageData(file);
    rays_ = computeRays();
    collisions_ = findIntersections();
    std::vector<unsigned char> output = calculateIlluminations();
    return std::make_pair(std::make_pair(output, png_), std::make_pair(w_, h_));
}

bool findShadows(Geometry light_source, Geometry collided_object, Vector3d intersection, Ray primary, bool debug) {
    if (std::holds_alternative<Sun>(light_source.geometry)) {
        // make and cast new secondary ray starting from intersection going towards light
        Sun light = std::get<Sun>(light_source.geometry);
        Ray secondary = Ray{primary.points, primary.scalars, intersection, light.direction};

        for (Geometry &obj : objects_) {
            if (std::holds_alternative<Sphere>(obj.geometry)) {
                secondary.origin = intersection + epsilon_ * light.direction.normalized();
                Sphere sphere = std::get<Sphere>(obj.geometry);
                // check if the sphere is the same as collided_obj
                if (std::holds_alternative<Sphere>(collided_object.geometry)) {
                    Sphere collided_sphere = std::get<Sphere>(collided_object.geometry);
                    if (sphere == collided_sphere) {
                        continue;   // ignore this obj
                    }
                }
                // check if secondary ray hits anything else other than collided_obj
                if (debug) {
                    std::cout << "Checking for intersection with " << sphere.center.transpose() << '\n';
                }
                auto out = raySphereIntersection(secondary, std::get<Sphere>(obj.geometry));
                if (out.has_value()) {
                    if (debug) {
                        std::cout << "Shadow was found\n";
                    }
                    return true;
                }
            // planes (elective)
            } else if (std::holds_alternative<Plane>(obj.geometry)) {
                secondary.origin = intersection + epsilon_ * light.direction.normalized(); // shadow ray epsilon
                Plane plane = std::get<Plane>(obj.geometry);
                // check if the plane is the same as collided_obj
                if (std::holds_alternative<Plane>(collided_object.geometry)) {
                    Plane collided_plane = std::get<Plane>(collided_object.geometry);
                    if (plane == collided_plane) {
                        continue;   // ignore this obj
                    }
                }
                // check if secondary ray hits anything else other than collided_obj
                if (debug) {
                    std::cout << "Checking for intersection with " << plane.normal.transpose() << '\n';
                }
                auto out = rayPlaneIntersection(secondary, std::get<Plane>(obj.geometry));
                if (out.has_value()) {
                    if (debug) {
                        std::cout << "Shadow was found\n";
                    }
                    return true;
                }
            // triangles (elective)
            } else if (std::holds_alternative<Triangle>(obj.geometry)) {
                secondary.origin = intersection + epsilon_ * light.direction.normalized(); // shadow ray epsilon
                Triangle tri = std::get<Triangle>(obj.geometry);
                // check if the plane is the same as collided_obj
                if (std::holds_alternative<Triangle>(collided_object.geometry)) {
                    Triangle collided_tri = std::get<Triangle>(collided_object.geometry);
                    if (tri == collided_tri) {
                        continue;   // ignore this obj
                    }
                }
                // check if secondary ray hits anything else other than collided_obj
                if (debug) {
                    std::cout << "Checking for intersection with " << tri.normal.transpose() << '\n';
                }
                auto out = rayTriangleIntersection(secondary, std::get<Triangle>(obj.geometry));
                if (out.has_value()) {
                    if (debug) {
                        std::cout << "Shadow was found\n";
                    }
                    return true;
                }
            }
        }
    }
    
    // bulb (elective)
    if (std::holds_alternative<Bulb>(light_source.geometry)) {
        // make and cast new secondary ray starting from intersection going towards bulb's location
        Bulb bulb = std::get<Bulb>(light_source.geometry);
        Vector3d toBulb = bulb.position - intersection;
        Ray secondary = Ray{primary.points, primary.scalars, intersection, toBulb};

        if (std::holds_alternative<Sphere>(collided_object.geometry)) {
            Sphere collided_sphere = std::get<Sphere>(collided_object.geometry);

            // check if collided object is behind bulb, skip shadows if it is
            Vector3d v = collided_sphere.center - bulb.position;
            Vector3d fromBulb = intersection - bulb.position;
            if (fromBulb.dot(v) < 0.0) {
                return false;   // object is behind bulb
            }
        } else { std::cerr << "Bulb test cases can only include spheres"; exit(1); }

        for (Geometry &obj : objects_) {
            if (std::holds_alternative<Sphere>(obj.geometry)) {
                secondary.origin = intersection + epsilon_ * toBulb.normalized(); // shadow ray epsilon
                Sphere sphere = std::get<Sphere>(obj.geometry);
                // check if the sphere is the same as collided_obj
                if (std::holds_alternative<Sphere>(collided_object.geometry)) {
                    Sphere collided_sphere = std::get<Sphere>(collided_object.geometry);
                    if (sphere == collided_sphere) {
                        continue;   // ignore this obj
                    }
                } else { std::cerr << "Bulb test cases can only include spheres"; exit(1); }

                // check if secondary ray hits anything else other than collided_obj
                auto out = raySphereIntersection(secondary, std::get<Sphere>(obj.geometry));
                if (out.has_value()) {
                    // check if object is further than the bulb
                    double distToObj = out.value().first;
                    double distToBulb = toBulb.norm();
                    if (distToObj < distToBulb) {
                        return true;    // object is closer than bulb, shadow was found
                    }
                }
            }
        }
    }
    return false;
}

std::vector<unsigned char> calculateIlluminations(bool debug) {
    std::vector<unsigned char> image(w_ * h_ * 4, 0);

    // anti-aliasing (elective)
    if (aa_ > 0) {
        int idx = 0;
        unsigned n_rays = 0;
        Vector3d pixel(0, 0, 0);
        for (const Ray &ray : rays_) {
            if (aa_collisions_[idx].collided_obj != nullptr) {
                // get the object that the ray collided with and its intersection point
                Collision c = aa_collisions_[idx];
                Geometry collided_object = *(c.collided_obj);
                Vector3d p = c.intersection;
                n_rays++;

                // calculate illumination for this pixel
                if (std::holds_alternative<Sphere>(collided_object.geometry)) {
                    Sphere collided_sphere = std::get<Sphere>(collided_object.geometry);
                    // loop through every light source
                    for (const Geometry &light_source : objects_) {
                        // process shadows
                        if (findShadows(light_source, collided_object, p, ray)) {
                            continue;
                        }
                        // perform lighting
                        Vector3d normal = (p - collided_sphere.center) / (p - collided_sphere.center).norm();
                        auto out = lambert(light_source, normal, ray, collided_sphere.color);
                        if (out.has_value()) {
                            pixel += out.value();
                        }
                    }
                } else if (std::holds_alternative<Plane>(collided_object.geometry)) {
                    Plane collided_plane = std::get<Plane>(collided_object.geometry);
                    // loop through every light source
                    for (const Geometry &light_source : objects_) {
                        // process shadows
                        if (findShadows(light_source, collided_object, p, ray)) {
                            continue;
                        }

                        // process lighting
                        auto out = lambert(light_source, collided_plane.normal, ray, collided_plane.color);
                        if (out.has_value()) {
                            pixel += out.value();
                        }
                    }
                } else if (std::holds_alternative<Triangle>(collided_object.geometry)) {
                    Triangle collided_tri = std::get<Triangle>(collided_object.geometry);
                    // loop through every light source
                    for (const Geometry &light_source : objects_) {
                        // process shadows
                        if (findShadows(light_source, collided_object, p, ray)) {
                            continue;
                        }

                        // process lighting
                        auto out = lambert(light_source, collided_tri.normal, ray, collided_tri.color);
                        if (out.has_value()) {
                            pixel += out.value();
                        }
                    }
                }

                // global illumination (elective)
                if (gi_ > 0) {
                    pixel += gi(p, *(c.collided_obj));
                }
            }

            // compute anti-aliased pixel (bring it all together!)
            if (idx % aa_ == 0 && idx != 0) {
                // average the colors for the final pixel
                if (n_rays == 0) {
                    pixel = Vector3d(0, 0, 0);
                } else {
                    pixel *= (1.0 / static_cast<double>(n_rays));
                }

                // clamp and convert pixel from RGB to sRGB
                auto clamp = [](double n) -> double { return std::max(0.0, std::min(1.0, n)); };
                auto sRGB = [](double L) -> double { return L > 0.0031308 ? 1.055*pow(L, 1/2.4) - 0.055 : 12.92*L; };
                pixel = (pixel.unaryExpr(clamp)).unaryExpr(sRGB);
                
                int x = ray.points.first, y = ray.points.second;
                image[((y * w_) + x) * 4 + 0] = pixel.x() * 255;
                image[((y * w_) + x) * 4 + 1] = pixel.y() * 255;
                image[((y * w_) + x) * 4 + 2] = pixel.z() * 255;
                image[((y * w_) + x) * 4 + 3] = alphas_[{x, y}];  // alpha
                n_rays = 0;

                pixel = Vector3d(0, 0, 0);
            }
            idx++;
        }

        return image;
    }

    // normal illumination (diffuse)
    int alpha = -1;
    for (const Ray &ray : rays_) {
        Vector3d pixel(0, 0, 0);

        if (collisions_.find(ray.points) != collisions_.end() && !ray.skip) {
            alpha = 255;
            // get the object that the ray collided with and its intersection point
            Geometry collided_object = *(collisions_[ray.points].collided_obj);
            Vector3d p = collisions_[ray.points].intersection;

            // calculate illumination for this pixel
            if (std::holds_alternative<Sphere>(collided_object.geometry)) {
                Sphere collided_sphere = std::get<Sphere>(collided_object.geometry);
                // loop through every light source
                for (const Geometry &light_source : objects_) {
                    // process shadows
                    if (findShadows(light_source, collided_object, p, ray)) {
                        continue;
                    }

                    // textures (elective)
                    if (collided_sphere.texture != nullptr) {
                        double u = collided_sphere.texture->texcoord[ray.points].first;
                        double v = collided_sphere.texture->texcoord[ray.points].second;
                        int x = static_cast<int>((u - floor(u)) * collided_sphere.texture->w);
                        int y = static_cast<int>((1 - (v - floor(v))) * collided_sphere.texture->h);

                        // need to convert texture map's sRGB pixels -> linear pixels
                        auto linear = [](double s) -> double { return s <= 0.04045 ? s / 12.92 : pow((s + 0.055) / 1.055, 2.4); };
                        double r = linear(collided_sphere.texture->data[(y * collided_sphere.texture->w + x) * 4 + 0] / 255.0);
                        double g = linear(collided_sphere.texture->data[(y * collided_sphere.texture->w + x) * 4 + 1] / 255.0);
                        double b = linear(collided_sphere.texture->data[(y * collided_sphere.texture->w + x) * 4 + 2] / 255.0);

                        collided_sphere.color = Vector3d(r, g, b);
                    }

                    // shininess (elective)
                    if (!shiny_.isApprox(Vector3d(0, 0, 0))) {
                        collided_sphere.color = shiny(collided_sphere, ray);
                    }

                    // perform lighting
                    Vector3d normal = (p - collided_sphere.center) / (p - collided_sphere.center).norm();
                    auto out = lambert(light_source, normal, ray, collided_sphere.color);
                    if (out.has_value()) {
                        pixel += out.value();
                    }
                }
            // planes (elective)
            } else if (std::holds_alternative<Plane>(collided_object.geometry)) {
                Plane collided_plane = std::get<Plane>(collided_object.geometry);
                // loop through every light source
                for (const Geometry &light_source : objects_) {
                    // process shadows
                    if (findShadows(light_source, collided_object, p, ray)) {
                        continue;
                    }

                    // process lighting
                    auto out = lambert(light_source, collided_plane.normal, ray, collided_plane.color);
                    if (out.has_value()) {
                        pixel += out.value();
                    }
                }
            // triangles (elective)
            } else if (std::holds_alternative<Triangle>(collided_object.geometry)) {
                Triangle collided_tri = std::get<Triangle>(collided_object.geometry);
                // loop through every light source
                for (const Geometry &light_source : objects_) {
                    // process shadows
                    if (findShadows(light_source, collided_object, p, ray)) {
                        continue;
                    }

                    // textures (elective)
                    Vector3d tricolor;
                    if (collided_tri.texture != nullptr) {
                        double u = collided_tri.texture->texcoord[ray.points].first;
                        double v = collided_tri.texture->texcoord[ray.points].second;
                        int x = static_cast<int>(u * collided_tri.texture->w);
                        int y = static_cast<int>(v * collided_tri.texture->h);

                        // need to convert texture map's sRGB pixels -> linear pixels
                        auto linear = [](double s) -> double { return s <= 0.04045 ? s / 12.92 : pow((s + 0.055) / 1.055, 2.4); };
                        double r = linear(collided_tri.texture->data[(y * collided_tri.texture->w + x) * 4 + 0] / 255.0);
                        double g = linear(collided_tri.texture->data[(y * collided_tri.texture->w + x) * 4 + 1] / 255.0);
                        double b = linear(collided_tri.texture->data[(y * collided_tri.texture->w + x) * 4 + 2] / 255.0);

                        tricolor = Vector3d(r, g, b);
                    } else {
                        tricolor = collided_tri.color;    // no texture; use normal color
                    }

                    // process lighting
                    auto out = lambert(light_source, collided_tri.normal, ray, tricolor);
                    if (out.has_value()) {
                        pixel += out.value();
                    }
                }
            }

            // exposure (elective)
            if (expose_.first) {
                double v = expose_.second;
                auto exposure = [v](double linear) -> double { return 1 - exp(-v * linear); };
                pixel = pixel.unaryExpr(exposure);
            }

            // clamp and convert pixel from RGB to sRGB
            auto clamp = [](double n) -> double { return std::max(0.0, std::min(1.0, n)); };
            auto sRGB = [](double L) -> double { return L > 0.0031308 ? 1.055*pow(L, 1/2.4) - 0.055 : 12.92*L; };
            pixel = (pixel.unaryExpr(clamp)).unaryExpr(sRGB);
        } else {
            alpha = 0;  // no collisions and/or skipped ray generation
        }

        // send pixel to final scene
        // std::cout << pixel.transpose() << " ";
        int x = ray.points.first, y = ray.points.second;
        image[((y * w_) + x) * 4 + 0] = pixel.x() * 255;
        image[((y * w_) + x) * 4 + 1] = pixel.y() * 255;
        image[((y * w_) + x) * 4 + 2] = pixel.z() * 255;
        image[((y * w_) + x) * 4 + 3] = alpha;  // alpha
    }

    if (debug) {
        constexpr int N_ROWS = 25;  /* number of img rows for debug */
        constexpr int N_COLS = 10;  /* number of img cols for debug */
        auto format = [](int n) -> string {
            return n == 0 ? "0  " : std::to_string(n) += std::string((static_cast<int>(log10(abs(n))) - 2)*-1, ' '); 
        };
        for (int x = 0; x < N_ROWS; x++) {
            for (int y = 0; y < N_COLS; y++) {
                std::cout << format((int) image[((y * w_) + x) * 4 + 0]) << " ";
                std::cout << format((int) image[((y * w_) + x) * 4 + 1]) << " ";
                std::cout << format((int) image[((y * w_) + x) * 4 + 2]) << " ";
                std::cout << format((int) image[((y * w_) + x) * 4 + 3]) << "\t";
            }
            std::cout << '\n';
        }
        std::cout << "and " << h_ - N_ROWS << " more rows, " << w_ - N_COLS << " more columns per row\n";
    }
    return image;
}

std::optional<Vector3d> lambert(const Geometry &light, const Vector3d &n, const Ray &ray, const Vector3d &objcolor) {
    // invert the normal before lighting
    Vector3d normal = n;
    normal = (ray.direction.normalized()).dot(normal) > 0 ? -1 * n : n;

    if (std::holds_alternative<Sun>(light.geometry)) {
        Sun sun = std::get<Sun>(light.geometry);
        Vector3d lambertDot = (objcolor.cwiseProduct(sun.color) * std::max(normal.dot(sun.direction.normalized()), 0.0));
        return lambertDot;
    // bulb (elective)
    } else if (std::holds_alternative<Bulb>(light.geometry)) {
        Bulb bulb = std::get<Bulb>(light.geometry);

        Vector3d p = collisions_[ray.points].intersection;
        Vector3d toLight = bulb.position - p;
        double distanceSquared = toLight.squaredNorm();
        double attenuation = 1.0 / distanceSquared;
        Vector3d lightDir = toLight.normalized();
        double lambertian = std::max(normal.dot(lightDir), 0.0) * attenuation;
        Vector3d lambertDot = (objcolor.cwiseProduct(bulb.color) * lambertian);
        return lambertDot;
    }
    return std::nullopt;
}

Vector3d gi(const Vector3d &intersection, const Geometry &obj) {
    /* calculates a randomly selected point for GI ray's direction */
    auto gi_dir = [](const Vector3d &p, const Geometry &o) -> Vector3d {
        std::random_device rd;
        std::default_random_engine rand(rd());
        std::uniform_real_distribution<double> unif(0, 1);
        double azim = unif(rand) * 2.0 * M_PI;          // aka random angle from 0 to 360 deg
        double incl = acos(sqrt(1.0 - unif(rand)));     // inclination from normal

        // get intersected normal
        Vector3d normal(0, 0, 0);
        if (std::holds_alternative<Sphere>(o.geometry)) {
            Sphere s = std::get<Sphere>(o.geometry);
            normal = (p - s.center) / (p - s.center).norm();
        } else if (std::holds_alternative<Plane>(o.geometry)) {
            Plane pl = std::get<Plane>(o.geometry);
            normal = pl.normal;
        } else if (std::holds_alternative<Triangle>(o.geometry)) {
            Triangle tri = std::get<Triangle>(o.geometry);
            normal = tri.normal;
        } else { std::cerr << "GI: unrecognized geometry\n"; exit(1); }

        Vector3d tangent = Vector3d(normal.y(), -normal.x(), 0).normalized();   // vector perpendicular to normal
        Vector3d bitangent = normal.cross(tangent).normalized();                // vector perpendicular to normal & tangent
        return sin(incl) * cos(azim) * tangent + sin(incl) * sin(azim) * bitangent + cos(incl) * normal;
    };

    // initial GI ray
    Ray gi_ray = Ray{{0,0}, {0.0,0.0}, intersection, gi_dir(intersection, obj), false};
    unsigned depth = 0;
    Vector3d gi_color(0, 0, 0);

    // collect indirect illuminations until depth max or no new intersections
    double t = std::numeric_limits<double>::max();
    Vector3d new_intersection = Vector3d(0, 0, 0);
    Geometry new_obj = obj;
    while (!(depth > gi_)) {
        Geometry old_obj = new_obj;
        // for every object in scene, check for an intersection w/ GI ray
        for (const Geometry &o : objects_) {
            if (std::holds_alternative<Sphere>(o.geometry)) {
                // avoid self-intersection
                if (std::holds_alternative<Sphere>(new_obj.geometry)) {
                    if (std::get<Sphere>(o.geometry) == std::get<Sphere>(new_obj.geometry)) { continue; }
                }
                auto out = raySphereIntersection(gi_ray, std::get<Sphere>(o.geometry));
                if (out.has_value()) {
                    // check if this collision is closer than the current closest
                    std::pair<double, Vector3d> pair = out.value();
                    if (t > pair.first) {
                        // retrieve the new intersected object
                        t = pair.first;
                        new_intersection = pair.second;
                        new_obj = o;
                    }
                }
            } else if (std::holds_alternative<Plane>(o.geometry)) {
                // avoid self-intersection
                if (std::holds_alternative<Plane>(new_obj.geometry)) {
                    if (std::get<Plane>(o.geometry) == std::get<Plane>(new_obj.geometry)) { continue; }
                }
                auto out = rayPlaneIntersection(gi_ray, std::get<Plane>(o.geometry));
                if (out.has_value()) {
                    // check if this collision is closer than the current closest
                    std::pair<double, Vector3d> pair = out.value();
                    if (t > pair.first) {
                        // retrieve the new intersected object
                        t = pair.first;
                        new_intersection = pair.second;
                        new_obj = o;
                    }
                }
            } else if (std::holds_alternative<Triangle>(o.geometry)) {
                // avoid self-intersection
                if (std::holds_alternative<Triangle>(new_obj.geometry)) {
                    if (std::get<Triangle>(o.geometry) == std::get<Triangle>(new_obj.geometry)) { continue; }
                }
                auto out = rayTriangleIntersection(gi_ray, std::get<Triangle>(o.geometry));
                if (out.has_value()) {
                    // check if this collision is closer than the current closest
                    std::pair<double, Vector3d> pair = out.value();
                    if (t > pair.first) {
                        // retrieve the new intersected object
                        t = pair.first;
                        new_intersection = pair.second;
                        new_obj = o;
                    }
                }
            }
        }

        // end early if no intersection (old_obj == new_obj)
        if (std::holds_alternative<Sphere>(old_obj.geometry) && std::holds_alternative<Sphere>(new_obj.geometry)) {
            if (std::get<Sphere>(old_obj.geometry) == std::get<Sphere>(new_obj.geometry)) { break; }
        } else if (std::holds_alternative<Plane>(old_obj.geometry) && std::holds_alternative<Plane>(new_obj.geometry)) {
            if (std::get<Plane>(old_obj.geometry) == std::get<Plane>(new_obj.geometry)) { break; }
        } else if (std::holds_alternative<Triangle>(old_obj.geometry) && std::holds_alternative<Triangle>(new_obj.geometry)) {
            if (std::get<Triangle>(old_obj.geometry) == std::get<Triangle>(new_obj.geometry)) { break; }
        }

        // calculate light contribution vector and add to summation vector (gi_color)
        if (std::holds_alternative<Sphere>(new_obj.geometry)) {
            Sphere s = std::get<Sphere>(new_obj.geometry);
            Vector3d spherenormal = (new_intersection - s.center) / (new_intersection - s.center).norm();
            for (const Geometry &light_source : objects_) {
                // process shadows
                if (findShadows(light_source, new_obj, new_intersection, gi_ray)) { 
                    continue;
                }
                // lighting
                auto out = lambert(light_source, spherenormal, gi_ray, s.color);
                if (out.has_value()) {
                    gi_color += out.value();
                }
            }
            // gi_color += s.color;
        } else if (std::holds_alternative<Plane>(new_obj.geometry)) {
            Plane pl = std::get<Plane>(new_obj.geometry);
            for (const Geometry &light_source : objects_) {
                // process shadows
                if (findShadows(light_source, new_obj, new_intersection, gi_ray)) { 
                    continue; 
                }
                // lighting
                auto out = lambert(light_source, pl.normal, gi_ray, pl.color);
                if (out.has_value()) {
                    gi_color += out.value();
                }
            }
            // gi_color += pl.color;
        } else if (std::holds_alternative<Triangle>(new_obj.geometry)) {
            Triangle tri = std::get<Triangle>(new_obj.geometry);
            for (const Geometry &light_source : objects_) {
                // process shadows
                if (findShadows(light_source, new_obj, new_intersection, gi_ray)) { 
                    continue; 
                }
                // lighting
                auto out = lambert(light_source, tri.normal, gi_ray, tri.color);
                if (out.has_value()) {
                    gi_color += out.value();
                }
            }
            // gi_color += tri.color;
        }

        gi_ray = Ray{{0,0}, {0.0,0.0}, new_intersection, gi_dir(new_intersection, new_obj), false};
        
        // increment depth
        t = std::numeric_limits<double>::max();
        depth++;
    }
    if (depth < 0) { std::cerr << "GI depth value is undefined\n"; exit(1); }
    return gi_color;
}

Vector3d shiny(const Sphere &obj, const Ray &ray, unsigned depth) {
    // recursion limiter
    if (depth > bounce_) {
        return Vector3d(0, 0, 0);
    }

    // for every sphere in scene, check for an intersection w/ reflection ray (this is modified from findIntersections)
    double distance = std::numeric_limits<double>::max();
    Vector3d intersection = Vector3d(0, 0, 0);
    std::shared_ptr<Sphere> newobj = nullptr;
    for (const Geometry &sphere : objects_) {
        if (std::holds_alternative<Sphere>(sphere.geometry)) {
            // prevent self-intersection
            if (obj == std::get<Sphere>(sphere.geometry)) {
                continue;
            }

            auto out = raySphereIntersection(ray, std::get<Sphere>(sphere.geometry));
            if (out.has_value()) {
                // check if this collision is closer than the current closest
                std::pair<double, Vector3d> pair = out.value();
                if (distance > pair.first) {
                    // retrieve the new intersected object
                    distance = pair.first;
                    intersection = pair.second;
                    newobj = std::make_shared<Sphere>(std::get<Sphere>(sphere.geometry));
                }
            }
        }
    }
    if (newobj == nullptr) {
        // no intersection found, end recursion stack
        return Vector3d(0, 0, 0);
    }

    // calculate reflection ray
    Vector3d N = ((intersection - newobj->center) / (intersection - newobj->center).norm()).normalized();
    Vector3d I = ray.direction.normalized();
    Vector3d r = I - 2 * (N.dot(I)) * N;
    Ray reflection = Ray{std::make_pair(0,0), std::make_pair(0.0,0.0), intersection, r, false};

    // recurse
    Vector3d reflectionColor = shiny(*newobj, reflection, depth + 1);

    // shininess calculation
    Vector3d objectColor = newobj->color;
    Vector3d shininess = newobj->shininess;
    
    Vector3d color = (Vector3d(1, 1, 1) - shininess).cwiseProduct(objectColor) + shininess.cwiseProduct(reflectionColor);
    return color;
}

std::map<std::pair<int, int>, Collision> findIntersections(bool debug) {
    std::map<std::pair<int, int>, Collision> collisions;

    // anti-aliasing (elective)
    int i = 1;
    int ray_i = 0;
    int n_collisions = 0;
    if (aa_ > 0) {
        std::vector<Collision> c(rays_.size(), Collision{std::numeric_limits<double>::max(), Vector3d(0, 0 ,0), nullptr});
        aa_collisions_ = c;
    }

    for (const Ray &ray : rays_) {
        Collision closest = {std::numeric_limits<double>::max(), Vector3d(0, 0 ,0), nullptr};
        for (Geometry &obj : objects_) {
            if (std::holds_alternative<Sphere>(obj.geometry)) {
                // ray-sphere intersection
                auto out = raySphereIntersection(ray, std::get<Sphere>(obj.geometry));
                if (out.has_value()) {
                    // check if this collision is closer than the current closest
                    std::pair<double, Vector3d> pair = out.value();
                    if (closest.distance > pair.first) {
                        if (debug && closest.distance != std::numeric_limits<double>::max()) {
                            std::cout << "Closer collision found\n";
                        }
                        closest.distance = pair.first;
                        closest.intersection = pair.second;
                        closest.collided_obj = &obj;
                    }
                    if (debug) {
                        std::cout << "Intersection found with distance " << pair.first << " at point ("
                                  << pair.second.transpose() << ")\n";
                    }
                } else {
                    // no intersection
                    if (debug) {
                        std::cout << "No intersection found.\n";
                    }
                }
            // planes (elective)
            } else if (std::holds_alternative<Plane>(obj.geometry)) {
                // ray-plane intersection
                auto out = rayPlaneIntersection(ray, std::get<Plane>(obj.geometry));
                if (out.has_value()) {
                    // check if this collision is closer than the current closest
                    std::pair<double, Vector3d> pair = out.value();
                    if (closest.distance > pair.first) {
                        if (debug && closest.distance != std::numeric_limits<double>::max()) {
                            std::cout << "Closer collision found\n";
                        }
                        closest.distance = pair.first;
                        closest.intersection = pair.second;
                        closest.collided_obj = &obj;
                    }
                    if (debug) {
                        std::cout << "Intersection found with distance " << pair.first << " at point ("
                                  << pair.second.transpose() << ")\n";
                    }
                } else {
                    // no intersection
                    if (debug) {
                        std::cout << "No intersection found.\n";
                    }
                }
            // triangles (elective)
            } else if (std::holds_alternative<Triangle>(obj.geometry)) {
                // ray-triangle intersection
                auto out = rayTriangleIntersection(ray, std::get<Triangle>(obj.geometry));
                if (out.has_value()) {
                    // check if this collision is closer than the current closest
                    std::pair<double, Vector3d> pair = out.value();
                    if (closest.distance > pair.first) {
                        if (debug && closest.distance != std::numeric_limits<double>::max()) {
                            std::cout << "Closer collision found\n";
                        }
                        closest.distance = pair.first;
                        closest.intersection = pair.second;
                        closest.collided_obj = &obj;
                    }
                    if (debug) {
                        std::cout << "Intersection found with distance " << pair.first << " at point ("
                                  << pair.second.transpose() << ")\n";
                    }
                } else {
                    // no intersection
                    if (debug) {
                        std::cout << "No intersection found.\n";
                    }
                }
            }
        }
        // add to collisions; if no k-v pair is found, there was no intersection
        if (aa_ > 0) {
            // anti-aliasing rays can have multiple collisions per pixel
            if (closest.distance != std::numeric_limits<double>::max()) {
                n_collisions++;
                aa_collisions_[ray_i] = closest;
            }

            // pre-compute alpha value of pixel by counting how many collisions in pixel
            if (i == aa_) {
                // sanity check (each key must be unique)
                if (alphas_.find(ray.points) != alphas_.end()) { std::cerr << "key exists in alphas_\n"; exit(1); }
                // add alpha value and reset counter
                alphas_[ray.points] = ((static_cast<double>(n_collisions) / static_cast<double>(aa_)) * 255.0);
                i = 0;
                n_collisions = 0;
            }
        } else {
            if (closest.distance != std::numeric_limits<double>::max()) {
                collisions[{ray.points.first, ray.points.second}] = closest;
            }
        }
        i++;
        ray_i++;
    }

    if (debug) {
        for (auto &pair : collisions) {
            std::cout << "Collision of ray (" << pair.first.first << ", " << pair.first.second << "), t=" <<
            pair.second.distance << " at point (" << pair.second.intersection.transpose() << ")\n";
        }
    }

    if (aa_ > 0) {
        // sanity check (must be a single alpha value for every pixel)
        if (alphas_.size() != (w_ * h_)) { std::cerr << "alphas_ is not wxh size\n"; exit(1); }

        // anti-aliasing (elective)
        int count = 0;
        for (const Collision &c : aa_collisions_) {
            if (c.collided_obj != nullptr) {
                count++;
            }
        }
        std::cout << count << " collisions recorded\n";
    } else {
        std::cout << collisions.size() << " collisions recorded\n";
    }

    return collisions;
}

std::optional<std::pair<double, Vector3d>> rayTriangleIntersection(const Ray &ray, const Triangle &triangle) {
    // inputs
    Vector3d r_o = ray.origin;
    Vector3d r_d = ray.direction.normalized();
    Vector3d n = triangle.normal;

    // process
    Vector3d p = triangle.v1->position;
    double t = ((p - r_o).dot(n)) / (r_d.dot(n));
    if (t > 0.0) {
        Vector3d p = r_o + t * r_d;
        // check if intersected point is in triangle
        std::pair<Vector3d, bool> bary = barycentric(p, triangle);
        if (bary.second) {
            // textures (elective)
            if (triangle.texture != nullptr) {
                Vector3d b = bary.first;
                double u1 = triangle.v1->texcoord.first, v1 = triangle.v1->texcoord.second;
                double u2 = triangle.v2->texcoord.first, v2 = triangle.v2->texcoord.second;
                double u3 = triangle.v3->texcoord.first, v3 = triangle.v3->texcoord.second;
                double u = b.x() * u1 + b.y() * u2 + b.z() * u3;
                double v = b.x() * v1 + b.y() * v2 + b.z() * v3;
                triangle.texture->texcoord[ray.points] = std::make_pair(u, v);
            }
            return std::make_pair(t, p);
        }
    }

    // no intersection
    return std::nullopt;
}

std::pair<Vector3d, bool> barycentric(const Vector3d &p, const Triangle &triangle) {
    double b2 = (triangle.e2).dot(p - triangle.v1->position);
    double b1 = (triangle.e1).dot(p - triangle.v1->position);
    double b0 = 1 - b1 - b2;

    // a point is in a triangle if all Barycentric coordinates b0, b1, b2 are in 0 <= b <= 1
    bool inTriangle = ((b0 >= 0.0) && (b0 <= 1.0) && (b1 >= 0.0) && (b1 <= 1.0) && (b2 >= 0.0) && (b2 <= 1.0));
    return std::make_pair(Vector3d(b0, b1, b2), inTriangle);
}

std::optional<std::pair<double, Vector3d>> rayPlaneIntersection(const Ray &ray, const Plane &plane) {
    // inputs
    Vector3d r_o = ray.origin;
    Vector3d r_d = ray.direction.normalized();
    Vector3d n = plane.normal;

    // process
    Vector3d p = (-plane.d * n) / (n.x()*n.x() + n.y()*n.y() + n.z()*n.z());
    double t = ((p - r_o).dot(n)) / (r_d.dot(n));
    if (t > 0.0) {
        Vector3d p = r_o + t * r_d;
        return std::make_pair(t, p);
    }

    // no intersection
    return std::nullopt;
}

std::optional<std::pair<double, Vector3d>> raySphereIntersection(const Ray &ray, const Sphere &sphere) {
    // inputs
    Vector3d r_o = ray.origin;
    Vector3d r_d = ray.direction.normalized();
    Vector3d c = sphere.center;
    double r = sphere.radius;

    // process
    bool inside = (pow((c - r_o).norm(), 2) < pow(r, 2));
    double t_c = ((c - r_o).dot(r_d)) / (r_d.norm());
    if (!inside && t_c < 0.0) {
        // no intersection
        return std::nullopt;
    }
    double d_2 = pow((r_o + t_c*r_d - c).norm(), 2);
    if (!inside && pow(r, 2) < d_2) {
        // no intersection
        return std::nullopt;
    }
    double t_offset = sqrt(pow(r, 2) - d_2) / (r_d.norm());
    double t = inside ? t_c + t_offset : t_c - t_offset;
    Vector3d p = t*r_d + r_o;
    
    // textures (elective)
    if (sphere.texture != nullptr) {
        std::pair<double, double> coord = textureCoordinates(p, sphere);
        sphere.texture->texcoord[ray.points] = coord;
    }
    return std::make_pair(t, p);
}

std::pair<double, double> textureCoordinates(const Vector3d &p, const Sphere &sphere) {
    Vector3d n = (1.0 / sphere.radius) * (p - sphere.center);
    double longitude = atan2(n.z(), n.x()), latitude = atan2(n.y(), sqrt(n.x() * n.x() + n.z() * n.z()));
    double u = 1.0 - (longitude + M_PI) / (2.0 * M_PI);
    double v = (latitude + M_PI / 2.0) / M_PI;
    return std::make_pair(u, v);
}

std::vector<Ray> computeRays(bool debug) {
    std::vector<Ray> rays;
    int max_dim = std::max(w_, h_);
    for (int x = 0; x < w_; x++) {
        for (int y = 0; y < h_; y++) {
            if (aa_ > 0) {
                // anti-aliasing (elective)
                std::uniform_real_distribution<double> unif_x(x-0.5, x+0.5);
                std::uniform_real_distribution<double> unif_y(y-0.5, y+0.5);
                std::random_device rd_x;
                std::default_random_engine random_x(rd_x());
                std::random_device rd_y;
                std::default_random_engine random_y(rd_y());

                for (int i = 0; i < aa_; i++) {
                    // stochastic sampling
                    double rand_x = unif_x(random_x);
                    double rand_y = unif_y(random_y);
                    double s_x = (2.0 * rand_x - w_) / static_cast<double>(max_dim);
                    double s_y = (h_ - 2.0 * rand_y) / static_cast<double>(max_dim);

                    // compute aa ray
                    Vector3d direction = f_ + s_x * r_ + s_y * u_;
                    direction.normalize();
                    
                    if (focus_ > 0.0 && lens_ > 0.0) {
                        // depth-of-field (elective)

                        // random point on disk of radius lens
                        std::uniform_real_distribution<double> unif(0.0, 1.0);
                        std::random_device rd;
                        std::default_random_engine rand(rd());
                        double radius = sqrt(unif(rand)) * lens_;
                        double angle = unif(rand) * 2 * M_PI;
                        std::pair<double, double> randpoint = std::make_pair(radius * cos(angle), radius * sin(angle));

                        // calculate new origin and direction
                        Ray r = Ray{std::make_pair(x,y), std::make_pair(s_x,s_y), e_, direction, false};
                        Vector3d p_focal = r.origin + focus_ * r.direction.normalized();
                        Vector3d o_new = e_ + randpoint.first * r_ + randpoint.second * u_;
                        Vector3d d_new = (p_focal - o_new).normalized();

                        // perturb ray with new origin and direction
                        r.origin = o_new;
                        r.direction = d_new;
                        rays.emplace_back(r);
                    } else {
                        rays.emplace_back(Ray{std::make_pair(x,y), std::make_pair(s_x,s_y), e_, direction, false});
                    }
                }
            } else {
                double s_x = (2.0 * x - w_) / static_cast<double>(max_dim);
                double s_y = (h_ - 2.0 * y) / static_cast<double>(max_dim);

                // fisheye (elective)
                if (fisheye_) {
                    if (s_x * s_x + s_y * s_y > 1.0) {
                        // make a dummy ray that does not shoot
                        rays.emplace_back(Ray{std::make_pair(x,y), std::make_pair(s_x,s_y), e_, Vector3d(0,0,0), true});
                        continue;
                    }
                    // compute fisheye ray
                    double modifier = sqrt(1 - s_x * s_x - s_y * s_y);
                    Vector3d direction = f_ * modifier + s_x * r_ + s_y * u_;
                    direction.normalize();
                    rays.emplace_back(Ray{std::make_pair(x,y), std::make_pair(s_x,s_y), e_, direction, false});
                // panorama (elective)
                } else if (panorama_) {
                    // compute panorama ray
                    double nx = (2.0 * x / w_) - 1;
                    double ny = (2.0 * y / h_) - 1;
                    double longitude = nx * M_PI;
                    double latitude = -ny * M_PI_2;
                    Vector3d direction(cos(latitude) * sin(longitude), sin(latitude), cos(latitude) * cos(longitude));
                    direction = f_ * direction.z() + u_ * direction.y() + r_ * direction.x();
                    direction.normalize();
                    rays.emplace_back(Ray{std::make_pair(x,y), std::make_pair(nx,ny), e_, direction, false});
                } else {
                    // compute normal ray
                    Vector3d direction = f_ + s_x * r_ + s_y * u_;
                    direction.normalize();
                    rays.emplace_back(Ray{std::make_pair(x,y), std::make_pair(s_x,s_y), e_, direction, false});
                }
            }
        }
    }
    if (debug) {
        for (const Ray &ray : rays) {
            std::cout << "Ray (" << ray.points.first << ", " << ray.points.second << ")\t";
            std::cout << "e: " << "(" << ray.origin.transpose() << ")" 
                      << "\tdirection: " << "(" << ray.direction.transpose() << ")" << '\n';
        }
    }
    std::cout << rays.size() << " rays computed\n";
    return rays;
}

std::vector<Geometry> processImageData(std::ifstream& file, bool debug) {
    std::vector<Geometry> objects;
    string line;

    int line_n = 1;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            std::istringstream iss(line);
            string cmd;
            iss >> cmd;

            if (cmd == "color") {
                string r, g ,b;
                iss >> r >> g >> b;
                color_ << stod(r), stod(g), stod(b);
                if (debug) {
                    std::cout << "Color changed to " << "(" << color_.transpose() << ")" << '\n';
                }
            } else if (cmd == "sphere") {
                string x, y, z, r;
                iss >> x >> y >> z >> r;
                Vector3d center(stod(x), stod(y), stod(z));

                // textures (elective)
                if (textures_.empty() || textures_.top().data.empty()) {
                    // no texture, default behavior
                    Geometry sphere = Geometry{Sphere{center, color_, stod(r), nullptr, shiny_}};
                    objects.push_back(sphere);
                } else {
                    std::shared_ptr<Texture> T = std::make_shared<Texture>(textures_.top());
                    Geometry sphere = Geometry{Sphere{center, color_, stod(r), T, shiny_}};
                    objects.push_back(sphere);
                }
            } else if (cmd == "sun") {
                string x, y, z;
                iss >> x >> y >> z;
                Vector3d direction(stod(x), stod(y), stod(z));
                Geometry sun = Geometry{Sun{direction, color_}};
                objects.push_back(sun);
            } else if (cmd == "bulb") {
                // bulb (elective)
                string x, y, z;
                iss >> x >> y >> z;
                Vector3d position(stod(x), stod(y), stod(z));
                Geometry bulb = Geometry{Bulb{position, color_}};
                objects.push_back(bulb);
            } else if (cmd == "plane") {
                // planes (elective)
                string a, b, c, d;
                iss >> a >> b >> c >> d;
                Vector3d normal(stod(a), stod(b), stod(c));
                double dist = stod(d);
                Geometry plane = Geometry{Plane{normal, dist, color_}};
                objects.push_back(plane);
            } else if (cmd == "xyz") {
                // triangles (elective)
                string x, y, z;
                iss >> x >> y >> z;
                Vector3d p(stod(x), stod(y), stod(z));
                vertices_.push_back(std::make_shared<Vertex>(Vertex{p, texcoord_}));
            } else if (cmd == "tri") {
                // triangles (elective)
                string i_1, i_2, i_3;
                iss >> i_1 >> i_2 >> i_3;
                auto reverse = [](int idx) -> int { return idx < 0 ? vertices_.size() + idx : idx - 1; };
                int idx_1 = reverse(stoi(i_1)), idx_2 = reverse(stoi(i_2)), idx_3 = reverse(stoi(i_3));
                std::shared_ptr<Vertex> v1 = vertices_.at(idx_1), v2 = vertices_.at(idx_2), v3 = vertices_.at(idx_3);

                // pre-compute e vectors and normal for Barycentric coordinates
                Vector3d normal = (v2->position - v1->position).cross(v3->position - v1->position);
                
                Vector3d a_1 = (v3->position - v1->position).cross(normal);
                Vector3d vec = v2->position - v1->position;
                Vector3d e_1 = (1.0 / a_1.dot(vec)) * a_1;

                Vector3d a_2 = (v2->position - v1->position).cross(normal);
                vec = v3->position - v1->position;
                Vector3d e_2 = (1.0 / a_2.dot(vec)) * a_2;

                // textures (elective)
                if (textures_.empty() || textures_.top().data.empty()) {
                    // no texture, default behavior
                    Geometry tri = Geometry{Triangle{v1, v2, v3, normal.normalized(), e_1, e_2, color_, nullptr}};
                    objects.push_back(tri);
                } else {
                    std::shared_ptr<Texture> T = std::make_shared<Texture>(textures_.top());
                    Geometry tri = Geometry{Triangle{v1, v2, v3, normal.normalized(), e_1, e_2, color_, T}};
                    objects.push_back(tri);
                }
            } else if (cmd == "texture") {
                // textures (elective)
                string filename;
                iss >> filename;
                Texture t = Texture{std::vector<unsigned char>(), 0, 0, {}};
                if (filename != "none") {
                    // open texture file
                    unsigned error = lodepng::decode(t.data, t.w, t.h, filename);
                    if (error) {
                        std::cerr << "Texture map error " << error << ": " << lodepng_error_text(error) << '\n';
                        exit(1);
                    }
                }
                textures_.push(t);
            } else if (cmd == "texcoord") {
                // textures (elective)
                string u, v;
                iss >> u >> v;
                texcoord_ = std::make_pair(stod(u), stod(v));
            } else if (cmd == "expose") {
                // exposure (elective)
                string v;
                iss >> v;
                expose_.first = true;
                expose_.second = stod(v);
            } else if (cmd == "up") {
                // view (elective)
                string x, y, z;
                iss >> x >> y >> z;
                u_ = Vector3d(stod(x), stod(y), stod(z));
            } else if (cmd == "eye") {
                // view (elective)
                string x, y, z;
                iss >> x >> y >> z;
                e_ = Vector3d(stod(x), stod(y), stod(z));
            } else if (cmd == "forward") {
                // view (elective)
                string x, y, z;
                iss >> x >> y >> z;
                f_ = Vector3d(stod(x), stod(y), stod(z));
                u_ = (r_.cross(f_)).normalized();
                r_ = (f_.cross(u_)).normalized();
            } else if (cmd == "fisheye") {
                // fisheye (elective)
                fisheye_ = true;
            } else if (cmd == "panorama") {
                // panorama (elective)
                panorama_ = true;
            } else if (cmd == "shininess") {
                // shininess (elective)
                string s_r, s_g, s_b;
                iss >> s_r >> s_g >> s_b;
                s_g = s_g.empty() ? s_r : s_g;
                s_b = s_b.empty() ? s_r : s_b;
                shiny_ = Vector3d(stod(s_r), stod(s_g), stod(s_b));
            } else if (cmd == "bounces") {
                // shininess (elective)
                string d;
                iss >> d;
                bounce_ = stoi(d);
            } else if (cmd == "aa") {
                // anti-aliasing (elective)
                string aa;
                iss >> aa;
                aa_ = stoi(aa);
            } else if (cmd == "dof") {
                // depth-of-field (elective)
                string focus, lens;
                iss >> focus >> lens;
                focus_ = stod(focus);
                lens_ = stod(lens);
            } else if (cmd == "gi") {
                // global illumination (elective)
                string gi;
                iss >> gi;
                gi_ = stoi(gi);
            } else {
                // comments or undefined commands
                // std::cerr << "undefined behavior\n"; exit(1);
            }
            // @TODO: add more commands (elective)
        }
        line_n++;
    }

    if (debug) {
        for (const auto &obj : objects) {
            std::visit(GeometryPrinter{}, obj.geometry);
        }
    }
    std::cout << objects.size() << " objects found\n";
    return objects;
}

void setImageGlobals(std::ifstream& file) {
    string line;
    std::getline(file, line);

    std::istringstream iss(line);
    string word;
    int i = 0;
    while (iss >> word) {
        if (i != 0) {
            if (i == 1) {
                w_ = stoi(word);
            } else if (i == 2) {
                h_ = stoi(word);
            } else if (i == 3) {
                png_ = word;
            } else {
                std::cerr << "First line had undefined behavior.\n";
                exit(1);
            }
        }
        i++;
    }
}

std::ifstream openFile(int argc, char** argv, bool debug) {
    if (argc < 2) {
        std::cerr << "No filename was provided.\n";
        exit(1);
    }
    std::ifstream file(argv[1]);
    if (!file.is_open()) {
        std::cerr << "Could not open file.\n";
        exit(1);
    }
    if (debug) {
        string line;
        std::cout << "File '" << argv[1] << "' opened successfully.\n";
        while (std::getline(file, line)) {
            std::cout << line << '\n';
        }
    }
    return file;
}