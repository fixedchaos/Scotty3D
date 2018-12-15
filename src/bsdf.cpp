#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::min;
using std::max;
using std::swap;

namespace CMU462 {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return albedo * (1.0 / PI);
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Implement DiffuseBSDF
	double Xi1 = (double)(std::rand()) / RAND_MAX;
	double Xi2 = (double)(std::rand()) / RAND_MAX;

	double theta = asin(sqrt(Xi1));
	double phi = 2.0 * PI * Xi2;

	double xs = sqrt(Xi1) * cosf(phi);
	double ys = sqrt(Xi1) * sinf(phi);
	double zs = sqrt(1 - Xi1);

	*wi = Vector3D(xs, ys, zs).unit();

	*pdf = abs(cos(theta)) / (PI);
	return f(wo, *wi);
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return reflectance * (1.0 / abs(wo.z));
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Implement MirrorBSDF
	reflect(wo, wi);
	*pdf = 1.f;
  return f(wo,*wi);
}

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
	return transmittance * (1.f / abs(wo.z));
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi,
                                  float* pdf) {
  // TODO (PathTracer):
  // Implement RefractionBSDF
	bool flag = refract(wo, wi, ior);
	if (flag) {
		*pdf = 1.f;
		return f(wo, *wi);
	}
	return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
	float ita_i, ita_t;
	if (wo.x == -wi.x && wo.y == -wi.y && wo.z == wi.z) {
		return reflectance * (1.f / abs(wo.z));
	}
	else {
		if (wo.z > 0) {
			ita_i = 1.0;
			ita_t = ior;
		}
		else {
			ita_i = ior;
			ita_t = 1.0;
		}
		return ((ita_t*ita_t) / (ita_i*ita_i)) * transmittance * (1.f / abs(wo.z));
	}
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO (PathTracer):
  // Compute Fresnel coefficient and either reflect or refract based on it.
	if (refract(wo, wi, ior))
	{
		double ita_i, ita_t, cos_theta_t, cos_theta_i;
		cos_theta_i = abs((*wi).z);
		cos_theta_t = abs(wo.z);
		if (wo.z > 0) {
			ita_i = 1.0;
			ita_t = ior;
		}
		else {
			ita_i = ior;
			ita_t = 1.0;
		}
		double r_para = (ita_t*cos_theta_i - ita_i * cos_theta_t) / (ita_t*cos_theta_i + ita_i * cos_theta_t);
		double r_perp = (ita_i*cos_theta_i - ita_t * cos_theta_t) / (ita_i*cos_theta_i + ita_t * cos_theta_t);
		float Fr = 0.5*(r_para*r_para + r_perp * r_perp);
		float Xi = (float)(std::rand()) / RAND_MAX;
		if (Xi < Fr) {
			reflect(wo, wi);
			*pdf = Fr;
		}
		else {
			*pdf = 1.f - Fr;
		}
	}
	else
	{
		reflect(wo, wi);
		*pdf = 1.f;
	}
  return *pdf * f(wo,*wi);
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // TODO (PathTracer):
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
	*wi = Vector3D(-wo.x, -wo.y, wo.z).unit();
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
  // TODO (PathTracer):
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
	double sin_theta_i, sin_theta_o;
	sin_theta_o = sqrt(1 - wo.z*wo.z);
	if (wo.z > 0)
	{
		sin_theta_i = sin_theta_o / ior;
	}
	else
	{
		sin_theta_i = sin_theta_o * ior;
	}

	if (sin_theta_i > 1)
	{
		return false;
	}

	if (wo.z > 0) {
		(*wi).z = -sqrt(1 - sin_theta_i * sin_theta_i);
	}
	else {
		(*wi).z = sqrt(1 - sin_theta_i * sin_theta_i);
	}
	if (sin_theta_o > 0) {
		(*wi).x = -wo.x*sin_theta_i / sin_theta_o;
		(*wi).y = -wo.y*sin_theta_i / sin_theta_o;
	}
	else {
		(*wi).x = 0;
		(*wi).y = 0;
	}
	(*wi) = (*wi).unit();
  return true;
}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi = sampler.get_sample(pdf);
  return Spectrum();
}

}  // namespace CMU462
