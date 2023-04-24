#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  double a = dot(r.d, r.d);
  double b = dot(2 * (r.o-o), r.d);
  double c = dot((r.o-o), (r.o-o)) - (this->r * this->r);
  
  t1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  t2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
  
  bool valid1 = t1 > r.min_t && t1 < r.max_t;
  bool valid2 = t2 > r.min_t && t2 < r.max_t;
  
  if (!valid1 && !valid2) {
    // Two invalid roots
    return false;
  }
  if (((valid1 && valid2) && (t1 > t2)) || (!valid1 && valid2)) {
    // Make sure t1 is smaller
    double temp = t1;
    t1 = t2;
    t2 = temp;
  }
  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  
  double t1, t2 = 0; // The two intersection points
  if (test(r, t1, t2)) {
    // Has intersection, return the closer intersection
    r.max_t = t1;
    return true;
  }
  // No intersection
  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2 = 0;
  if (test(r, t1, t2)) {
    // Update intersection
    r.max_t = t1;
    i->t = t1;
    Vector3D normalv = r.at_time(t1) - o;
    normalv.normalize();
    i->n = normalv;
    i->primitive = this;
    i->bsdf = get_bsdf();
    return true;
  }
  // No intersection
  return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
