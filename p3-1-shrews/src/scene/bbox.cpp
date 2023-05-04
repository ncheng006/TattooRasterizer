#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

    bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

        // TODO (Part 2.2):
        // Implement ray - bounding box intersection test
        // If the ray intersected the bouding box within the range given by
        // t0, t1, update t0 and t1 with the new intersection times.

        auto tx1 = (min.x - r.o.x) / r.d.x;
        auto tx2 = (max.x - r.o.x) / r.d.x;
        auto ty1 = (min.y - r.o.y) / r.d.y;
        auto ty2 = (max.y - r.o.y) / r.d.y;
        auto tz1 = (min.z - r.o.z) / r.d.z;
        auto tz2 = (max.z - r.o.z) / r.d.z;
        if (tx1 > tx2) {
            auto temp = tx1;
            tx1 = tx2;
            tx2 = temp;
        }

        if (ty1 > ty2) {
            auto temp = ty1;
            ty1 = ty2;
            ty2 = temp;
        }

        if (tz1 > tz2) {
            auto temp = tz1;
            tz1 = tz2;
            tz2 = temp;
        }

        t0 = std::max(tx1, std::max(ty1, tz1));
        t1 = std::min(tx2, std::min(ty2, tz2));

        if (tx1 > ty2 || ty1 > tx2 || t0 > tz2 || tz1 > t1) {
            return false;
        }

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

