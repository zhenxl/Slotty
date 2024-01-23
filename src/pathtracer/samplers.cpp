
#include "samplers.h"
#include "../util/rand.h"
#include "../scene/shape.h"
#include <iostream>

constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers {

Vec2 Rect::sample(RNG &rng) const {
	//A3T1 - step 2 - supersampling

    // Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
    // Useful function: rng.unit()

    return Vec2{rng.unit() * size.x, rng.unit() * size.y};
}

float Rect::pdf(Vec2 at) const {
	if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y) return 0.0f;
	return 1.0f / (size.x * size.y);
}

Vec3 Point::sample(RNG &rng) const {
	return point;
}

float Point::pdf(Vec3 at) const {
	return at == point ? 1.0f : 0.0f;
}

Vec3 Triangle::sample(RNG &rng) const {
	float u = std::sqrt(rng.unit());
	float v = rng.unit();
	float a = u * (1.0f - v);
	float b = u * v;
	return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

float Triangle::pdf(Vec3 at) const {
	float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
	float u = 0.5f * cross(at - v1, at - v2).norm() / a;
	float v = 0.5f * cross(at - v2, at - v0).norm() / a;
	float w = 1.0f - u - v;
	if (u < 0.0f || v < 0.0f || w < 0.0f) return 0.0f;
	if (u > 1.0f || v > 1.0f || w > 1.0f) return 0.0f;
	return 1.0f / a;
}

Vec3 Hemisphere::Uniform::sample(RNG &rng) const {

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float theta = std::acos(Xi1);
	float phi = 2.0f * PI_F * Xi2;

	float xs = std::sin(theta) * std::cos(phi);
	float ys = std::cos(theta);
	float zs = std::sin(theta) * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Hemisphere::Uniform::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return 1.0f / (2.0f * PI_F);
}

Vec3 Hemisphere::Cosine::sample(RNG &rng) const {

	float phi = rng.unit() * 2.0f * PI_F;
	float cos_t = std::sqrt(rng.unit());

	float sin_t = std::sqrt(1 - cos_t * cos_t);
	float x = std::cos(phi) * sin_t;
	float z = std::sin(phi) * sin_t;
	float y = cos_t;

	return Vec3(x, y, z);
}

float Hemisphere::Cosine::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return dir.y / PI_F;
}

Vec3 Sphere::Uniform::sample(RNG &rng) const {
	//A3T7 - sphere sampler

    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform
	Vec3 dir = hemi.sample(rng);
	if(rng.coin_flip(0.5f)) {
		dir.y = -dir.y;
	}
    return dir;
}

float Sphere::Uniform::pdf(Vec3 dir) const {
	return 1.0f / (4.0f * PI_F);
}

Sphere::Image::Image(const HDR_Image& image) {
    //A3T7 - image sampler init

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
	auto size = image.data().size();
	_pdf.resize(size);
	_cdf.resize(size);
	for(size_t i = 0; i < size; i++) {
		size_t row_idx = i / w;
		float theta = PI_F - ((float)row_idx + 0.5f) / (float) h * PI_F;
		_pdf[i] = image.at(i).luma() * std::sin(theta);
		_cdf[i] = _pdf[i] + (i == 0? 0.0f: _cdf[i-1]);
	}
	float total = _cdf.back();
	for (size_t i = 0; i < size; ++i) {
		_pdf[i] /= total;
		_cdf[i] /= total;
    }
}

Vec3 Sphere::Image::sample(RNG &rng) const {
	if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Uniform::sample(rng);
		auto uniform_sampler = Uniform();
    	return uniform_sampler.sample(rng);
	} else {
		// Step 2: Importance sampling
		// Use your importance sampling data structure to generate a sample direction.
		// Tip: std::upper_bound
		float p = rng.unit();
		size_t i = std::upper_bound(_cdf.begin(), _cdf.end(), p) - _cdf.begin();
		size_t row_idx = i / w;
		size_t col_idx = i % w;
		float theta = PI_F - ((float)row_idx + 0.5f) / (float) h * PI_F;
		float phi  = ((float)col_idx + 0.5f) / (float) w * PI_F * 2;
		float y_s = std::cos(theta);
		float x_s = std::sin(theta) * std::cos(phi);
		float z_s = std::sin(theta) * std::sin(phi);
    	return Vec3{x_s, y_s, z_s};
	}
}

float Sphere::Image::pdf(Vec3 dir) const {
    if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its pdf
		auto uniform_sampler = Uniform();
    	return uniform_sampler.pdf(dir);
	} else {
		// A3T7 - image sampler importance sampling pdf
		// What is the PDF of this distribution at a particular direction?
		Vec2 uv = Shapes::Sphere::uv(dir);
		// uv.x is longitude phi, \in [0, 1)
        // uv.y is latitude theta, \in (0, 1) (south, north)
		size_t col_index = std::clamp((size_t)((float)w * uv.x- 0.5f), 0ul, w- 1ul);
		size_t rol_index = std::clamp((size_t)((float)h * (1 -uv.y)- 0.5f), 0ul, h- 1ul);
		size_t i = rol_index  * w + col_index;

		float jacobian = float(w * h) * 0.5f / PI_F / PI_F / std::sin(uv.y * PI_F);
        auto p = _pdf[i] * jacobian;

        return p;
		// auto uv = Shapes::Sphere::uv(dir);
		// // uv.x is longitude phi, \in [0, 1)
		// // uv.y is latitude theta, \in (0, 1) (south, north)

		// auto _h = std::clamp(size_t(uv.y * float(h)), 0ul, h - 1ul);
		// auto _w = std::clamp(size_t(uv.x * float(w)), 0ul, w - 1ul);
		// auto i = _h * w + _w;
		// float jacobian = float(w * h) * 0.5f / PI_F / PI_F / std::sin(uv.y * PI_F);
		// auto p = _pdf[i] * jacobian;

		// return p;
	}
}

} // namespace Samplers
