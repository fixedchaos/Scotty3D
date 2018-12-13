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
	switch (split_index % 3)
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

	++split_index;
	BVH_build_recusive(root->l, max_leaf_size);
	BVH_build_recusive(root->r, max_leaf_size);
	--split_index;
}

BVHAccel::~BVHAccel() {
	// TODO (PathTracer):
	// Implement a proper destructor for your BVH accelerator aggregate
	destruct_BVH_recursive(root);
}

void BVHAccel::destruct_BVH_recursive(BVHNode* node)
{
	if (root == nullptr)
	{
		return;
	}
	else
	{
		auto left = node->l;
		auto right = node->r;
		delete node;
		destruct_BVH_recursive(left);
		destruct_BVH_recursive(right);
	}
}

BBox BVHAccel::get_bbox() const { return root->bb; }

bool BVHAccel::intersect(const Ray &ray) const {
	// TODO (PathTracer):
	// Implement ray - bvh aggregate intersection test. A ray intersects
	// with a BVH aggregate if and only if it intersects a primitive in
	// the BVH that is not an aggregate.
	double t0, t1;
	bool intersection = root->bb.intersect(ray, t0, t1);
	double tmin = std::max(t0, ray.min_t),
		tmax = std::min(t1, ray.max_t);
	if (intersection && tmin < tmax)
	{
		return find_intersect_recursive(root, ray);
	}
	return false;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
	// TODO (PathTracer):
	// Implement ray - bvh aggregate intersection test. A ray intersects
	// with a BVH aggregate if and only if it intersects a primitive in
	// the BVH that is not an aggregate. When an intersection does happen.
	// You should store the non-aggregate primitive in the intersection data
	// and not the BVH aggregate itself.

	double t0, t1;
	if (root->bb.intersect(ray, t0, t1)) {
		if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
			return find_intersect_recursive(root, ray, isect);
		}
	}
}

bool BVHAccel::find_intersect_recursive(BVHNode* node, const Ray &ray) const
{
	if (node == nullptr)
	{
		return false;
	}

	if (node->isLeaf())
	{
		for (size_t i = 0; i < node->range; i++)
		{
			auto idx = node->start + i;
			if (primitives[idx]->intersect(ray))
			{
				return true;
			}
		}
	}
	else
	{
		auto left = node->l, right = node->r;
		double t0, t1;
		bool intersection = left->bb.intersect(ray, t0, t1);
		double tmin = std::max(t0, ray.min_t),
			tmax = std::min(t1, ray.max_t);
		if (intersection && tmin < tmax)
		{
			if (find_intersect_recursive(left, ray))
			{
				return true;
			}
		}

		intersection = right->bb.intersect(ray, t0, t1);
		tmin = std::max(t0, ray.min_t),
			tmax = std::min(t1, ray.max_t);
		if (intersection && tmin < tmax)
		{
			return find_intersect_recursive(right, ray);
		}
	}

	return false;
}

bool BVHAccel::find_intersect_recursive(BVHNode* node, const Ray &ray, Intersection *isect) const
{
	bool hit = false;

	if (node->isLeaf())
	{
		for (size_t i = 0; i < node->range; i++)
		{
			auto idx = node->start + i;
			if (primitives[idx]->intersect(ray, isect))
			{
				hit = true;
			}
		}
	}
	else
	{
		auto left = node->l, right = node->r;
		double t0, t1, t2, t3, tbest1, tbest2;
		bool left_intersection = left->bb.intersect(ray, t0, t1);
		bool right_intersection = right->bb.intersect(ray, t2, t3);

		if (t0 >= ray.min_t&&t0 <= ray.max_t) {
			tbest1 = t0;
		}
		else {
			tbest1 = t1;
		}
		if (t2 >= ray.min_t&&t2 <= ray.max_t) {
			tbest2 = t2;
		}
		else {
			tbest2 = t3;
		}
		BVHNode *first = (tbest1 <= tbest2) ? node->l : node->r;
		BVHNode *second = (tbest1 <= tbest2) ? node->r : node->l;

		if (first->bb.intersect(ray, t0, t1)) {
			if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t))) {
				if (find_intersect_recursive(first, ray, isect)) {
					hit = true;
				}
			}
		}

		if (second->bb.intersect(ray, t0, t1)) {
			if (!((t0 < ray.min_t&&t1 < ray.min_t) || (t0 > ray.max_t&&t1 > ray.max_t)) && isect->t > t0) {
				if (find_intersect_recursive(second, ray, isect)) {
					hit = true;
				}
			}
		}
	}
	return hit;
}

} // namespace StaticScene
}  // namespace CMU462
