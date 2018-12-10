#include "sphere.h"

#include <cmath>

#include "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CMU462 {
namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
	Vector3D o1 = r.o,
		o2 = o;

	double delta = pow(dot(o1 - o2, r.d)/r.d.norm(), 2) + pow(this->r, 2) - (o1 - o2).norm2();
	if (delta < 0)
	{
		return false;
	}
	else
	{
		t1 = (dot(o2 - o1, r.d) / r.d.norm() - sqrt(delta)) / r.d.norm();
		t2 = (dot(o2 - o1, r.d) / r.d.norm() + sqrt(delta)) / r.d.norm();
		return true;
	}
}

bool Sphere::intersect(const Ray& r) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
	double t1, t2;
	if (!test(r, t1, t2)) {
		return false;
	}
	else if((t1 >= r.min_t && t1 <=r.max_t) || (t2 >= r.min_t && t2 <= r.max_t))
	{
		return true;
	}
  return false;
}

bool Sphere::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
	double t1, t2;
	if (!test(r, t1, t2)) {
		return false;
	}
	else
	{
		if (t1 >= r.min_t&&t1 <= r.max_t) {
			isect->t = t1;
			Vector3D intersection_p = r.o + r.d*t1;
			isect->n = normal(intersection_p);
			isect->primitive = this;
			isect->bsdf = get_bsdf();
			r.max_t = t1;
			return true;
		}
		else if (t2 >= r.min_t&&t2 <= r.max_t) {
			isect->t = t2;
			Vector3D intersection_p = r.o + r.d*t2;
			isect->n = normal(intersection_p);
			isect->primitive = this;
			isect->bsdf = get_bsdf();
			r.max_t = t2;
			return true;
		}
	}
  return false;
}

void Sphere::draw(const Color& c) const { Misc::draw_sphere_opengl(o, r, c); }

void Sphere::drawOutline(const Color& c) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

}  // namespace StaticScene
}  // namespace CMU462
