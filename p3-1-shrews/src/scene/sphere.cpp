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

            auto a = dot(r.d, r.d);
            auto b = dot(2 * (r.o - this->o), r.d);
            auto c = dot(r.o - this->o, r.o - this->o) - this->r2;

            auto inside = b * b - 4.0 * a * c;
            if (inside < 0) {
                return false;
            }

            auto t_neg = (-b - sqrt(inside)) / (2.0 * a);
            auto t_pos = (-b + sqrt(inside)) / (2.0 * a);


            if (t_neg > r.min_t && t_pos < r.max_t) {
                t1 = t_neg;
                t2 = t_pos;
                r.max_t = t2;
                r.min_t = t1;
                return true;
            }
            else {
                return false;
            }

        }

        bool Sphere::has_intersection(const Ray &r) const {

            // TODO (Part 1.4):
            // Implement ray - sphere intersection.
            // Note that you might want to use the the Sphere::test helper here.
            double t1, t2;
            return this->test(r, t1, t2);

        }

        bool Sphere::intersect(const Ray &r, Intersection *i) const {

            // TODO (Part 1.4):
            // Implement ray - sphere intersection.
            // Note again that you might want to use the the Sphere::test helper here.
            // When an intersection takes place, the Intersection data should be updated
            // correspondingly.
            double t1, t2;
            bool intersect = this->test(r, t1, t2);
            if (intersect == false) {
                return false;
            }

            i->bsdf = this->get_bsdf();
            i->t = r.max_t;
            i->primitive = this;
            Vector3D norm = (r.min_t * r.d + r.o) - this->o;
            norm.normalize();
            i->n = norm;

            return true;
        }

        void Sphere::draw(const Color &c, float alpha) const {
            Misc::draw_sphere_opengl(o, r, c);
        }

        void Sphere::drawOutline(const Color &c, float alpha) const {
            // Misc::draw_sphere_opengl(o, r, c);
        }

    } // namespace SceneObjects
} // namespace CGL
