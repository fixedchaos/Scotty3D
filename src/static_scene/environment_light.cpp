#include "environment_light.h"

namespace CMU462 {
namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
  // TODO: (PathTracer) initialize things here as needed
	int width = envMap->w;
	int height = envMap->h;
	sampleToWorld[0] = Vector3D(0, 0, -1);
	sampleToWorld[1] = Vector3D(1, 0, 0);
	sampleToWorld[2] = Vector3D(0, 1, 0);
	worldToSample = sampleToWorld.T();
}

Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight, float* pdf) const {
  // TODO: (PathTracer) Implement
	double Xi1 = (double)(std::rand()) / RAND_MAX;
	double Xi2 = (double)(std::rand()) / RAND_MAX;

	double theta = acos(1 - 2 * Xi1);
	double phi = 2.0 * PI * Xi2;

	double xs = sinf(theta) * cosf(phi);
	double ys = sinf(theta) * sinf(phi);
	double zs = cosf(theta);

	*distToLight = INF_D;
	*pdf = 1.f / (4 * PI);
	*wi = sampleToWorld * Vector3D(xs, ys, zs);
	return sample_dir(Ray(p, *wi));
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  // TODO: (PathTracer) Implement
	const Vector3D& dir = (worldToSample * r.d).unit();

	double theta = acos(dir.z);
	double phi = atan2(dir.y, dir.x) + PI;
	double u = phi / (2.0*PI);
	double v = theta / PI;

	int width = envMap->w;
	int height = envMap->h;

	// bilinear interpolation
	float x = u * width - 0.5;
	float y = v * height - 0.5;
	int sx = int(floor(x));
	int sy = int(floor(y));
	float u_ratio = x - sx;
	float v_ratio = y - sy;
	float u_opposite = 1 - u_ratio;
	float v_opposite = 1 - v_ratio;
	int sx0, sx1, sy0, sy1;
	sx0 = std::max(0, sx);
	sy0 = std::max(0, sy);
	sx1 = std::min(width - 1, sx + 1);
	sy1 = std::min(height - 1, sy + 1);

	Spectrum data1 = envMap->data[sx0 + sy0 * width];
	Spectrum data2 = envMap->data[sx1 + sy0 * width];
	Spectrum data3 = envMap->data[sx0 + sy1 * width];
	Spectrum data4 = envMap->data[sx1 + sy1 * width];

	return (data1*u_opposite + data2 * u_ratio)*v_opposite + (data3*u_opposite + data4 * u_ratio)*v_ratio;
}

}  // namespace StaticScene
}  // namespace CMU462
