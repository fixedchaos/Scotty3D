#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 {
namespace StaticScene {


BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {
  this->primitives = _primitives;

  // TODO (PathTracer):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bb;
  for (size_t i = 0; i < primitives.size(); ++i) {
    bb.expand(primitives[i]->get_bbox());
  }

  root = new BVHNode(bb, 0, primitives.size());
  BVH_build_recusive(root, max_leaf_size);
}

void BVHAccel::BVH_build_recusive(BVHNode* root, size_t max_leaf_size)
{
	if (root->range <= max_leaf_size)
	{
		return;
	}
	// Find split using a simple strategy: x,y,z in cyclic
	static size_t split_index = 0;
	switch (split_index)
	{
	case 0:
		std::sort(primitives.begin() + root->start, primitives.begin() + root->start + root->range,
			[](const Primitive* lhs, const Primitive* rhs) { return lhs->get_bbox().centroid().x < rhs->get_bbox().centroid().x;  });
		break;
	case 1:
		std::sort(primitives.begin() + root->start, primitives.begin() + root->start + root->range,
			[](const Primitive* lhs, const Primitive* rhs) { return lhs->get_bbox().centroid().y < rhs->get_bbox().centroid().y;  });
		break;
	case 2:
		std::sort(primitives.begin() + root->start, primitives.begin() + root->start + root->range,
			[](const Primitive* lhs, const Primitive* rhs) { return lhs->get_bbox().centroid().z < rhs->get_bbox().centroid().z;  });
		break;
	default:
		break;
	}
	split_index = (split_index + 1) % 3;

	// With a simple model : middle to split
	auto mid = root->range / 2;

	// Compute left and right node bbox
	BBox left_bbox, right_bbox;
	for (size_t i = root->start; i < root->start + mid; i++)
	{
		left_bbox.expand(primitives[i]->get_bbox());
	}

	for (size_t i = root->start + mid; i < root->start + root->range; i++)
	{
		right_bbox.expand(primitives[i]->get_bbox());
	}

	root->l = new BVHNode(left_bbox, root->start, mid);
	root->r = new BVHNode(right_bbox, root->start + mid, root->range - mid);
	BVH_build_recusive(root->l, max_leaf_size);
	BVH_build_recusive(root->r, max_leaf_size);
}


BVHAccel::~BVHAccel() {
  // TODO (PathTracer):
  // Implement a proper destructor for your BVH accelerator aggregate

}

BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(const Ray &ray) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate.

  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray)) hit = true;
  }

  return hit;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
  // TODO (PathTracer):
  // Implement ray - bvh aggregate intersection test. A ray intersects
  // with a BVH aggregate if and only if it intersects a primitive in
  // the BVH that is not an aggregate. When an intersection does happen.
  // You should store the non-aggregate primitive in the intersection data
  // and not the BVH aggregate itself.

  bool hit = false;
  for (size_t p = 0; p < primitives.size(); ++p) {
    if (primitives[p]->intersect(ray, isect)) hit = true;
  }

  return hit;
}

}  // namespace StaticScene
}  // namespace CMU462
