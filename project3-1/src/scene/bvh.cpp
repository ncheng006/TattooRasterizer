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
  int leaf_size = 0;
  Vector3D centroids = Vector3D();

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    centroids += bb.centroid();
    bbox.expand(bb);
    leaf_size++;
  }

  BVHNode *node = new BVHNode(bbox);;

  // If there are no more than max_leaf_size primitives in the list;
  // create a leaf node.
  if (leaf_size <= max_leaf_size) {
    node->start = start;
    node->end = end;
    node->l = NULL;
    node->r = NULL;
    return node;
  } else { // otherwise, split the nodes into a left and right node recursively.

    // Find the longest axis
    double x = bbox.extent.x;
    double y = bbox.extent.y;
    double z = bbox.extent.z;
    
    double longest_axis = std::max(x, std::max(y, z));

    std::vector<Primitive *>::iterator mid_point;

    // Reorder primitives so that left nodes always come before right nodes.
    // Calculate split point according to the average of the centroids along that axis.
    if (x == longest_axis) {
      double c_x = centroids.x / (double) leaf_size;
      mid_point = std::partition(start, end, [c_x, leaf_size](Primitive *p) {
        return ((*p).get_bbox().centroid().x > c_x);
      });
    } else if (y == longest_axis) {
      double c_y = centroids.y / (double) leaf_size;
      mid_point = std::partition(start, end, [c_y, leaf_size](Primitive *p) {
        return ((*p).get_bbox().centroid().y > c_y);
      });
    } else {
      double c_z = centroids.z / (double) leaf_size;
      mid_point = std::partition(start, end, [c_z, leaf_size](Primitive *p) {
        return ((*p).get_bbox().centroid().z > c_z);
      });
    }

    BVHNode *left = construct_bvh(start, mid_point, max_leaf_size);
    BVHNode *right = construct_bvh(mid_point, end, max_leaf_size);

    node->l = left;
    node->r = right;
  }

  return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  double t0;
  double t1;

  if (node->bb.intersect(ray, t0, t1)) {
    if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        if ((*p)->has_intersection(ray)) {
          return true;
        }
      }
    } else {
      return has_intersection(ray, node->l) || has_intersection(ray, node->r);
    }
    
    return false;
  }
  
  return false;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  double t0 = ray.min_t;
  double t1 = ray.max_t;
  
  // Check every node for a bounding box intersection
  if (node->bb.intersect(ray, t0, t1)) {

    // For each leaf, check whether the primitives intersect with the ray
    // Primitive->intersect will update ray.min_t and ray.max_t for the next bbox call
    if (node->isLeaf()) {
      bool hit = false;
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        hit = (*p)->intersect(ray, i) || hit;
      }
      return hit;
    } else { // Otherwise, call intersect on both the left and right children
      bool l_isect = intersect(ray, i, node->l);
      bool r_isect = intersect(ray, i, node->r);
      
      // Cannot shortcircuit here
      return l_isect || r_isect;
    }
  }
  
  // If no intersection is found, there's no need to check further
  return false;
}

} // namespace SceneObjects
} // namespace CGL
