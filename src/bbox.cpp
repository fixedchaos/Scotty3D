#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CMU462 {

bool BBox::intersect(const Ray &r, double &t0, double &t1) const {
  // TODO (PathTracer):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bounding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.
	double txmin = (min.x - r.o.x) * r.inv_d.x;
	double txmax = (max.x - r.o.x) * r.inv_d.x;
	double tymin = (min.y - r.o.y) * r.inv_d.y;
	double tymax = (max.y - r.o.y) * r.inv_d.y;
	double tzmin = (min.z - r.o.z) * r.inv_d.z;
	double tzmax = (max.z - r.o.z) * r.inv_d.z;

	if (txmin > txmax) {
		std::swap(txmin, txmax);
	}
	if (tymin > tymax) {
		std::swap(tymin, tymax);
	}
	if (tzmin > tzmax) {
		std::swap(tzmin, tzmax);
	}

	double tmax = std::min(std::min(txmax, tymax), tzmax);
	double tmin = std::max(std::max(txmin, tymin), tzmin);
	if (tmax < tmin || tmax < 0)
	{
		return false;
	}

	t0 = tmin;
	t1 = tmax;
	return true;
}

void BBox::draw(Color c) const {
  glColor4f(c.r, c.g, c.b, c.a);

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

std::ostream &operator<<(std::ostream &os, const BBox &b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

}  // namespace CMU462
