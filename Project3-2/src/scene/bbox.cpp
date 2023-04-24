#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
  
  // TODO (Part 2.2):
  // Calculate intersections along the x-plane
  double t0_x = (min.x - r.o.x) / r.d.x;
  double t1_x = (max.x - r.o.x) / r.d.x;

  // Calculate intersections along the y-plane
  double t0_y = (min.y - r.o.y) / r.d.y;
  double t1_y = (max.y - r.o.y) / r.d.y;

  // Calculate intersections along the z-plane
  double t0_z = (min.z - r.o.z) / r.d.z;
  double t1_z = (max.z - r.o.z) / r.d.z;

  // Find the min and max values of each intersection
  double tmin_x = std::min(t0_x, t1_x);
  double tmin_y = std::min(t0_y, t1_y);
  double tmin_z = std::min(t0_z, t1_z);

  double tmax_x = std::max(t0_x, t1_x);
  double tmax_y = std::max(t0_y, t1_y);
  double tmax_z = std::max(t0_z, t1_z);

  // Final tmin is the maxes of the mins
  // Final tmax is the mins of the maxes
  double tmin = std::max(tmin_x, std::max(tmin_y, tmin_z));
  double tmax = std::min(tmax_x, std::max(tmax_y, tmax_z));

  // Check for invalid or suboptimal hit
  if ((tmin < r.min_t && tmax < r.min_t) || (tmin > r.max_t && tmax > r.max_t)) {
    return false;
  }

  // Check if the sets are disjoint
  if (tmin_x > tmax_y || tmin_y > tmax_x || tmin_z > tmax_x || tmin_x > tmax_z || tmin_z > tmax_y || tmin_y > tmax_z) {
    return false;
  }

  // Set t0 and t1 to tmin and tmax
  t0 = tmin;
  t1 = tmax;

  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
