
#include "shape.h"
#include "../geometry/util.h"
#include <iostream>

namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Sphere::hit(Ray ray) const {
	// std::cout << "now hit sphere" << std::endl;
	//A3T2 - sphere hit

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

	// float a = ray.dir.norm_squared();
	// float b = 2 * dot(ray.point, ray.dir);
	// float c = ray.point.norm_squared() - this->radius * this->radius;
	// float tmp = std::sqrt(b*b - 4 * a * c);
	// float t1 = (-b - tmp) / (2 * a);
	// float t2 = (-b + tmp) / (2 * a);

	// Vec3 p1 = ray.point + t1 * ray.dir;
	// float dis1 = ray.dir.norm() * t1;
	// Vec3 p2 = ray.point + t2 *ray.dir;
	// float dis2 = ray.dir.norm()  * t2;

	// Vec3 pos;
	// float dist;
	// float t;
	// (void)t;

	// if (dis2 < ray.dist_bounds.x || dis1 > ray.dist_bounds.y) {
	// 	 PT::Trace ret;
	// 	 ret.hit = false;
	// 	//  std::cout << "not hit" << std::endl;
	// 	 return ret;
	// }

	// if (dis1 < ray.dist_bounds.x && dis2 > ray.dist_bounds.y) {
	// 	 PT::Trace ret;
	// 	 ret.hit = false;
	// 	// std::cout << "not hit" << std::endl;
	// 	 return ret;
	// }

	// // if ((dis1 < ray.dist_bounds.x || dis1 > ray.dist_bounds.y) && (dis2 < ray.dist_bounds.x || dis2 > ray.dist_bounds.y)) {
	// // 	// PT::Trace ret;
	// // 	// ret.hit = false;
	// // 	// std:: cout << "not hit: " << t1 << " " << t2 <<  std::endl;
	// // 	// return ret;
	// // }else 
	// if (dis1 < ray.dist_bounds.x ) {
	// 	pos = p2;
	// 	dist = dis2;
	// 	 t = t2;
	// } else {
	// 	pos = p1;
	// 	dist = dis1;
	// 	t = t1;
	// }


    // PT::Trace ret;
    // ret.origin = ray.point;
    // ret.hit = true;       // was there an intersection?
	// // dist = 0.0;
    // ret.distance = dist;   // at what distance did the intersection occur?
    // ret.position = pos; // where was the intersection?
    // ret.normal   = pos;   // what was the surface normal at the intersection?
	// ret.uv = uv(ray.point + t * ray.dir); 	   // what was the uv coordinates at the intersection? (you may find Sphere::uv to be useful)
    // return ret;
	PT::Trace ret;
	ret.origin = ray.point;

	auto od = dot(ray.point, ray.dir);
	auto d_square = ray.dir.unit().norm_squared();
	auto discriminant = 4 * od * od - 4 * d_square * (ray.point.norm_squared() - radius * radius);
	if (discriminant < 0) {
	ret.hit = false;
	return ret;
	}
	auto half_root = std::sqrt(discriminant) / 2 * d_square;
	auto t = -od /d_square  - half_root;
	if (ray.dist_bounds.x > t || t > ray.dist_bounds.y)
	t = -od/d_square + half_root;
	if (ray.dist_bounds.x > t || t > ray.dist_bounds.y) {
	ret.hit = false;
	return ret;
  }

  ret.hit = true;       // was there an intersection?
  ret.distance = t;   // at what distance did the intersection occur?
  ret.position = ray.at(t); // where was the intersection?
  ret.normal = ret.position.unit();   // what was the surface normal at the intersection?
  ret.uv = uv(ray.point + t * ray.dir);
  return ret;
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::closed_sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}
