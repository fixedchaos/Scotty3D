#include "triangle.h"

#include "CMU462/CMU462.h"
#include "GL/glew.h"

namespace CMU462 {
namespace StaticScene {

Triangle::Triangle(const Mesh* mesh, vector<size_t>& v) : mesh(mesh), v(v) {}
Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3)
    : mesh(mesh), v1(v1), v2(v2), v3(v3) {}

BBox Triangle::get_bbox() const {
  // TODO (PathTracer):
  // compute the bounding box of the triangle

  return BBox();
}

bool Triangle::intersect(const Ray& r) const {
  // TODO (PathTracer): implement ray-triangle intersection
	Vector3D p0 = mesh->positions[v1];
	Vector3D p1 = mesh->positions[v2];
	Vector3D p2 = mesh->positions[v3];
	Vector3D e1 = p1 - p0,
		e2 = p2 - p0,
		s = r.o - p0,
		d = r.d;

	double c = dot(cross(e1, d), e2);
	if (c == 0)
	{
		return false;
	}
	else
	{
		double u = -dot(cross(s, e2), d) / c;
		double v = dot(cross(e1, d), s) / c;
		double t = -dot(cross(s, e2), e1) / c;
		double w = 1 - u - v;
		if (t >= r.min_t && t <= r.max_t && u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

  return false;
}

bool Triangle::intersect(const Ray& r, Intersection* isect) const {
  // TODO (PathTracer):
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly
	Vector3D p0 = mesh->positions[v1];
	Vector3D p1 = mesh->positions[v2];
	Vector3D p2 = mesh->positions[v3];
	Vector3D e1 = p1 - p0,
		e2 = p2 - p0,
		s = r.o - p0,
		d = r.d;

	double c = dot(cross(e1, d), e2);
	if (c == 0)
	{
		return false;
	}
	else
	{
		double u = -dot(cross(s, e2), d) / c;
		double v = dot(cross(e1, d), s) / c;
		double t = -dot(cross(s, e2), e1) / c;
		double w = 1 - u - v;
		if (t >= r.min_t && t <= r.max_t && u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1)
		{
			isect->t = t;
			isect->n = mesh->normals[v1] * w + mesh->normals[v2] * u + mesh->normals[v3] * v;
			if (dot(isect->n, d) > 0)
			{
				isect->n = -isect->n;
			}
			isect->primitive = this;
			isect->bsdf = get_bsdf();

			return true;
		}
		else
		{
			return false;
		}
	}
  return false;
}

void Triangle::draw(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_TRIANGLES);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

void Triangle::drawOutline(const Color& c) const {
  glColor4f(c.r, c.g, c.b, c.a);
  glBegin(GL_LINE_LOOP);
  glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
             mesh->positions[v1].z);
  glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
             mesh->positions[v2].z);
  glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
             mesh->positions[v3].z);
  glEnd();
}

}  // namespace StaticScene
}  // namespace CMU462
