#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
    namespace SceneObjects {

        BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                           size_t max_leaf_size) {

            primitives = std::vector<Primitive *>(_primitives);
            root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
        }

        BVHAccel::~BVHAccel() {
            if (root)
                delete root;
            primitives.clear();
        }

        BBox BVHAccel::get_bbox() const { return root->bb; }

        void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->draw(c, alpha);
                }
            } else {
                draw(node->l, c, alpha);
                draw(node->r, c, alpha);
            }
        }

        void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->drawOutline(c, alpha);
                }
            } else {
                drawOutline(node->l, c, alpha);
                drawOutline(node->r, c, alpha);
            }
        }

        BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                         std::vector<Primitive *>::iterator end,
                                         size_t max_leaf_size) {

            // TODO (Part 2.1):
            // Construct a BVH from the given vector of primitives and maximum leaf
            // size configuration. The starter code build a BVH aggregate with a
            // single leaf node (which is also the root) that encloses all the
            // primitives.


            BBox bbox;
            int size = 0;
            for (auto p = start; p != end; p++) {
                BBox bb = (*p)->get_bbox();
                bbox.expand(bb);
                size++;
            }
            BVHNode *node = new BVHNode(bbox);
            node->start = start;
            node->end = end;

            if (size <= max_leaf_size) {
                return node;
            }

            //find the longest axis
            Vector3D longest = bbox.extent;
            int axis = 0;
            if (longest.y > longest.x) {
                axis = 1;
            }
            if (longest.z > longest.y) {
                axis = 2;
            }

            //if x is longest
            if (axis == 0) {
                std::sort(start, end, [](Primitive *a, Primitive *b) {
                    return a->get_bbox().centroid().x < b->get_bbox().centroid().x;
                });
            }
            //if y is longest
            if (axis == 1) {
                std::sort(start, end, [](Primitive *a, Primitive *b) {
                    return a->get_bbox().centroid().y < b->get_bbox().centroid().y;
                });
            }
            //if z is longest
            if (axis == 2) {
                std::sort(start, end, [](Primitive *a, Primitive *b) {
                    return a->get_bbox().centroid().z < b->get_bbox().centroid().z;
                });
            }

            int mid = size / 2;
            node->l = construct_bvh(start, start + mid, max_leaf_size);
            node->r = construct_bvh(start + mid, end, max_leaf_size);
            return node;


        }

        bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // Take note that this function has a short-circuit that the
            // Intersection version cannot, since it returns as soon as it finds
            // a hit, it doesn't actually have to find the closest hit.
            double t0, t1;
            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    total_isects++;
                    if ((*p)->has_intersection(ray)) {
                        return true;
                    }
                }
                return false;
            }
            else {
                BVHNode* left = node->l;
                BVHNode* right = node->r;
                bool hit_left = has_intersection(ray, left);
                bool hit_right = has_intersection(ray, right);
                return hit_left || hit_right;
            }
            return false;
        }

        bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.

            double t0, t1;
            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }
            if (node->isLeaf()) {
                bool hit = false;
                for (auto p = node->start; p != node->end; p++) {
                    total_isects++;
                    if ((*p)->intersect(ray, i)) {
                        hit = true;
                    }
                }
                return hit;
            }
            else {
                BVHNode* left = node->l;
                BVHNode* right = node->r;
                bool hit_left = intersect(ray, i, left);
                bool hit_right = intersect(ray, i, right);
                return hit_left || hit_right;
            }
            return false;
        }

    } // namespace SceneObjects
} // namespace CGL
