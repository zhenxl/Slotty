// clang-format off
#include "pipeline.h"

#include <iostream>

#include "../lib/log.h"
#include "../lib/mathlib.h"
#include "sample_pattern.h"
#include "framebuffer.h"
#include <functional>

static uint32_t INSIDE_BIT = 0;
static uint32_t LEFT_BIT = 1 << 0;
static uint32_t RIGHT_BIT = 1 << 1;
static uint32_t BOTTOM_BIT = 1 << 2;
static uint32_t TOP_BIT = 1 << 3;
static uint32_t FAR_BIT = 1 << 5;
static uint32_t NEAR_BIT = 1 << 4;

template<PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(std::vector<Vertex> const& vertices,
                                                   typename Program::Parameters const& parameters,
                                                   Framebuffer* framebuffer_) {
	// Framebuffer must be non-null:
	assert(framebuffer_);
	auto& framebuffer = *framebuffer_;

	// A1T7: sample loop
	// TODO: update this function to rasterize to *all* sample locations in the framebuffer.
	//  	 This will probably involve inserting a loop of the form:
	// 		 	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	//      	for (uint32_t s = 0; s < samples.size(); ++s) { ... }
	//   	 around some subset of the code.
	// 		 You will also need to transform the input and output of the rasterize_* functions to
	// 	     account for the fact they deal with pixels centered at (0.5,0.5).

	std::vector<ShadedVertex> shaded_vertices;
	shaded_vertices.reserve(vertices.size());

	//--------------------------
	// shade vertices:
	for (auto const& v : vertices) {
		ShadedVertex sv;
		Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
		shaded_vertices.emplace_back(sv);
	}

	//--------------------------
	// assemble + clip + homogeneous divide vertices:
	std::vector<ClippedVertex> clipped_vertices;

	// reserve some space to avoid reallocations later:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		// clipping lines can never produce more than one vertex per input vertex:
		clipped_vertices.reserve(shaded_vertices.size());
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		// clipping triangles can produce up to 8 vertices per input vertex:
		clipped_vertices.reserve(shaded_vertices.size() * 8);
	}
	// clang-format off

	//coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
	//x: [-1,1] -> [0,width]
	//y: [-1,1] -> [0,height]
	//z: [-1,1] -> [0,1] (OpenGL-style depth range)
	Vec3 const clip_to_fb_scale = Vec3{
		framebuffer.width / 2.0f,
		framebuffer.height / 2.0f,
		0.5f
	};
	Vec3 const clip_to_fb_offset = Vec3{
		0.5f * framebuffer.width,
		0.5f * framebuffer.height,
		0.5f
	};


	// helper used to put output of clipping functions into clipped_vertices:
	auto emit_vertex = [&](ShadedVertex const& sv) {
		ClippedVertex cv;
		float inv_w = 1.0f / sv.clip_position.w;
		cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
		cv.inv_w = inv_w;
		cv.attributes = sv.attributes;
		clipped_vertices.emplace_back(cv);
	};

	// actually do clipping:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2) {
			clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3) {
			clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// rasterize primitives:

	std::vector<Fragment> fragments;
	std::vector<float> depth_buffer;
	// helper used to put output of rasterization functions into fragments:
	auto emit_fragment = [&](Fragment const& f) { fragments.emplace_back(f); };
	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	// actually do rasterization:
	for(uint32_t s = 0; s < samples.size(); s++) {
		auto& sample_offset = samples[s];
		for(ClippedVertex& v: clipped_vertices) {
			v.fb_position.x += sample_offset.x;
			v.fb_position.y += sample_offset.y;
		}
		if constexpr (primitive_type == PrimitiveType::Lines) {
			for (uint32_t i = 0; i + 1 < clipped_vertices.size(); i += 2) {
				rasterize_line(clipped_vertices[i], clipped_vertices[i + 1], emit_fragment);
			}
		} else if constexpr (primitive_type == PrimitiveType::Triangles) {
			for (uint32_t i = 0; i + 2 < clipped_vertices.size(); i += 3) {
				rasterize_triangle(clipped_vertices[i], clipped_vertices[i + 1], clipped_vertices[i + 2], emit_fragment);
			}
		} else {
			static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
		}
		for(ClippedVertex& v: clipped_vertices) {
			v.fb_position.x -= sample_offset.x;
			v.fb_position.y -= sample_offset.y;
		}
		//--------------------------
		// depth test + shade + blend fragments:
		uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer 
								// (indicates something is wrong with clipping)
	
		for (auto const& f : fragments) {

			// fragment location (in pixels):
			int32_t x = (int32_t)std::floor(f.fb_position.x);
			int32_t y = (int32_t)std::floor(f.fb_position.y);

			// if clipping is working properly, this condition shouldn't be needed;
			// however, it prevents crashes while you are working on your clipping functions,
			// so we suggest leaving it in place:
			if (x < 0 || (uint32_t)x >= framebuffer.width || 
				y < 0 || (uint32_t)y >= framebuffer.height) {
				++out_of_range;
				continue;
			}

			// local names that refer to destination sample in framebuffer:
			float& fb_depth = framebuffer.depth_at(x, y, s);
			Spectrum& fb_color = framebuffer.color_at(x, y, s);


			// depth test:
			if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always) {
				// "Always" means the depth test always passes.
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never) {
				// "Never" means the depth test never passes.
				continue; //discard this fragment
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less) {
				// "Less" means the depth test passes when the new fragment has depth less than the stored depth.
				// A1T4: Depth_Less
				// TODO: implement depth test! We want to only emit fragments that have a depth less than the stored depth, hence "Depth_Less".
				auto cur_fb_depth = f.fb_position.z;
				if (cur_fb_depth >=fb_depth) {
					continue;
				} else {
					fb_depth = cur_fb_depth;
				}

			} else {
				static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
			}

			// if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
			if constexpr (!(flags & Pipeline_DepthWriteDisableBit)) {
				fb_depth = f.fb_position.z;
			} 

			// shade fragment:
			ShadedFragment sf;
			sf.fb_position = f.fb_position;
			Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

			// write color to framebuffer if color writes aren't disabled:
			if constexpr (!(flags & Pipeline_ColorWriteDisableBit)) {
				// blend fragment:
				if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace) {
					fb_color = sf.color;
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add) {
					// A1T4: Blend_Add
					// TODO: framebuffer color should have fragment color multiplied by fragment opacity added to it.
					fb_color = fb_color + sf.opacity* sf.color; //<-- replace this line
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over) {
					// A1T4: Blend_Over
					// TODO: set framebuffer color to the result of "over" blending (also called "alpha blending") the fragment color over the framebuffer color, using the fragment's opacity
					// 		 You may assume that the framebuffer color has its alpha premultiplied already, and you just want to compute the resulting composite color
					fb_color = sf.color * sf.opacity + fb_color * (1 - sf.opacity); //<-- replace this line
				} else {
					static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
				}
			}
		}
		if (out_of_range > 0) {
			if constexpr (primitive_type == PrimitiveType::Lines) {
				warn("Produced %d fragments outside framebuffer; this indicates something is likely "
					"wrong with the clip_line function.",
					out_of_range);
			} else if constexpr (primitive_type == PrimitiveType::Triangles) {
				warn("Produced %d fragments outside framebuffer; this indicates something is likely "
					"wrong with the clip_triangle function.",
					out_of_range);
			}
		}
	}
}

// -------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const& a, ShadedVertex const& b, float t) -> ShadedVertex {
	ShadedVertex ret;
	ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}

/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  	va, vb: endpoints of line
 *  	emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 * 
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(ShadedVertex const& va, ShadedVertex const& vb,
                                      std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// Determine portion of line over which:
	// 		pt = (b-a) * t + a
	//  	-pt.w <= pt.x <= pt.w
	//  	-pt.w <= pt.y <= pt.w
	//  	-pt.w <= pt.z <= pt.w
	// ... as a range [min_t, max_t]:

	float min_t = 0.0f;
	float max_t = 1.0f;

	// want to set range of t for a bunch of equations like:
	//    a.x + t * ba.x <= a.w + t * ba.w
	// so here's a helper:
	auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr) {
		// restrict range such that:
		// l + t * dl <= r + t * dr
		// re-arranging:
		//  l - r <= t * (dr - dl)
		if (dr == dl) {
			// want: l - r <= 0
			if (l - r > 0.0f) {
				// works for none of range, so make range empty:
				min_t = 1.0f;
				max_t = 0.0f;
			}
		} else if (dr > dl) {
			// since dr - dl is positive:
			// want: (l - r) / (dr - dl) <= t
			min_t = std::max(min_t, (l - r) / (dr - dl));
		} else { // dr < dl
			// since dr - dl is negative:
			// want: (l - r) / (dr - dl) >= t
			max_t = std::min(max_t, (l - r) / (dr - dl));
		}
	};

	// local names for clip positions and their difference:
	Vec4 const& a = va.clip_position;
	Vec4 const& b = vb.clip_position;
	Vec4 const ba = b - a;

	// -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.x, ba.x);
	clip_range(a.x, ba.x, a.w, ba.w);
	// -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.y, ba.y);
	clip_range(a.y, ba.y, a.w, ba.w);
	// -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.z, ba.z);
	clip_range(a.z, ba.z, a.w, ba.w);

	if (min_t < max_t) {
		if (min_t == 0.0f) {
			emit_vertex(va);
		} else {
			ShadedVertex out = lerp(va, vb, min_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
		if (max_t == 1.0f) {
			emit_vertex(vb);
		} else {
			ShadedVertex out = lerp(va, vb, max_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
	}
}


template<PrimitiveType p, class P, uint32_t flags>
uint32_t Pipeline<p, P, flags>::GetClipCode(ShadedVertex const &a) {

	uint32_t code = INSIDE_BIT;
	Vec4 v = a.clip_position;

	if (v.x < -v.w) code |= LEFT_BIT;
	if (v.x > v.w) code |= RIGHT_BIT;
	if (v.y < -v.w) code |= BOTTOM_BIT;
	if (v.y > v.w) code |= TOP_BIT;
	if (v.z > v.w) code |= FAR_BIT;
	if (v.z < -v.w)code |= NEAR_BIT;

	return code;
}

template<PrimitiveType p, class P, uint32_t flags>
auto Pipeline<p, P, flags>::SutherlandHodgman_clip_triangle(const Vec4 &a, const Vec4 &b, const Vec4 &c, uint32_t code)-> Polygon {
	Polygon polygon{};
	polygon.SetFromTriangle(a, b, c);

	if (code & LEFT_BIT) {
		polygon = clip_plane(
			LEFT_BIT, polygon,
			[](const Vec4& v) { return v.x >= -v.w; },
			[](Vec4& v) { v.x = -v.w; });
	}

	if (code & RIGHT_BIT) {
		polygon = clip_plane(
			RIGHT_BIT, polygon,
			[](const Vec4& v) { return v.x <= v.w; },
			[](Vec4& v) { v.x = v.w; });
	}

	if (code & BOTTOM_BIT) {
		polygon = clip_plane(
			BOTTOM_BIT, polygon,
			[](const Vec4& v) { return v.y >= -v.w; },
			[](Vec4& v) { v.y = -v.w; });
	}

	if (code & TOP_BIT) {
		polygon = clip_plane(
			TOP_BIT, polygon,
			[](const Vec4& v) { return v.y <= v.w; },
			[](Vec4& v) { v.y = v.w; });
	}

	if (code & FAR_BIT) {
		polygon = clip_plane(
			FAR_BIT, polygon,
			[](const Vec4& v) { return v.z <= v.w; },
			[](Vec4& v) { v.z = v.w; });
	}

	if (code & NEAR_BIT) {
		polygon = clip_plane(
			NEAR_BIT, polygon,
			[](const Vec4& v) { return v.z >= -v.w; },
			[](Vec4& v) { v.z = -v.w; });
	}


	for(int i = 0 ; i < polygon.Size(); i++) {
		if (polygon[i].pos.w <= 0.0f) {
			polygon.Clear();
			break;
		}
	}
	return polygon;
}

CFG_FORCE_INLINE float Dot(uint32_t planeCode, const Vec4& v){
	if (planeCode & LEFT_BIT) return v.x + v.w; /* v * (1 0 0 1)  */
	if (planeCode & RIGHT_BIT) return v.x - v.w; /* v * (-1 0 0 1) */
	if (planeCode & BOTTOM_BIT) return v.y + v.w; /* v * (0 -1 0 1) */
	if (planeCode & TOP_BIT) return v.y - v.w; /* v * (0 1 0 1)  */
	if (planeCode & FAR_BIT) return v.z - v.w; /* v * (0 0 -1 0) */
	if (planeCode & NEAR_BIT) return v.z; /* v * (0 0 1 1)  */

	return INFINITY;
}

CFG_FORCE_INLINE float Point2PlaneDistance(
		uint32_t clipPlane, const Vec4& a, const Vec4& b) {
	{
		return Dot(clipPlane, a) / (Dot(clipPlane, a) - Dot(clipPlane, b));
	}
}



template<PrimitiveType p, class P, uint32_t flags>
CFG_FORCE_INLINE auto Pipeline<p, P, flags> :: clip_plane(uint32_t plane, const Polygon& inPolygon, const Predicate& isInside, const Clip& clip) -> Polygon {
	Polygon outPolygon;
	
	for(uint32_t i = 0, j = 1; i < inPolygon.Size(); i++, j++) {
		if (j == inPolygon.Size()) j = 0;

		const auto& aPos = inPolygon[i].pos;
		const auto& aDist = inPolygon[i].distance;

		// Point B on the segment
		const auto& bPos = inPolygon[j].pos;
		const auto& bDist = inPolygon[j].distance;

		float t = Point2PlaneDistance(plane, aPos, bPos);
		Vec4 newPos = aPos * (1 -t) + bPos * t;
		const Vec3 newDist = aDist * (1 - t) + bDist * t;
		clip(newPos);
		if (isInside(aPos)) {
			if (isInside(bPos)) {
				outPolygon.Add(typename Polygon::Point{bPos, bDist});
			} else {
				outPolygon.Add(typename Polygon::Point{newPos, newDist});
			}
		} else if (isInside(bPos)) {
			outPolygon.Add(typename Polygon::Point{ newPos, newDist });
			outPolygon.Add(typename Polygon::Point{ bPos, bDist });
		}
	}
	return outPolygon;
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  	va, vb, vc: vertices of triangle
 *  	emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 * 
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
	ShadedVertex const& va, ShadedVertex const& vb, ShadedVertex const& vc,
	std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// A1EC: clip_triangle
	// TODO: correct code!
	uint32_t clipCode0 = GetClipCode(va);
	uint32_t clipCode1 = GetClipCode(vb);
	uint32_t clipCode2 = GetClipCode(vc);

	if (clipCode0 | clipCode1 | clipCode2) {
		if (!(clipCode0 & clipCode1 & clipCode2)) {
			auto polygon = SutherlandHodgman_clip_triangle(
							va.clip_position,
							vb.clip_position,
							vc.clip_position,
							(clipCode0 ^ clipCode1) | (clipCode1 ^ clipCode2) | (clipCode2 ^ clipCode0));
			std::vector<const ShadedVertex> clip_buffer(polygon.Size());
			for(int j = 0; j < polygon.Size(); j++) {
				auto weight = polygon[j].distance;

				if (weight.x == 1.0f) {
					clip_buffer.push_back(va);

				} else if (weight.y == 1.0f) {
					clip_buffer.push_back(vb);
				} else if (weight.z == 1.0f) {
					clip_buffer.push_back(vc);
				} else {
					Vec4 pos = weight.x * va.clip_position + weight.y * vb.clip_position + weight.z * vc.clip_position;
					ShadedVertex newVertex;
					newVertex.clip_position = pos;
					if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
						newVertex.attributes = va.attributes;
					} else {
						for(uint32_t i = 0; i < newVertex.attributes.size(); i++) {
							newVertex.attributes[i] = weight.x * va.attributes[i] + weight.y * vb.attributes[i] + weight.z * vc.attributes[i];
						}
					}
					clip_buffer.push_back(newVertex);
				}
			}

			for (int j = 2; j < polygon.Size(); j++) {
				auto& v0 = clip_buffer[0];
				auto& v1 = clip_buffer[j-1];
				auto& v2 = clip_buffer[j];

				emit_vertex(v0);
				emit_vertex(v1);
				emit_vertex(v2);
			}
		}

	} else {
		emit_vertex(va);
		emit_vertex(vb);
		emit_vertex(vc);
	}
}

// -------------------------------------------------------------------------
// rasterization functions

/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 * 
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points. 
 * 
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces 
 *    a different rasterization result. 
 *    We will not explicitly check for 45 degree lines along the diamond edges (this will be extra credit),
 *    but you should be able to handle 45 degree lines in every other case (such as starting from pixel centers)
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
	ClippedVertex const& va, ClippedVertex const& vb,
	std::function<void(Fragment const&)> const& emit_fragment) {
	if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
		assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
	}
	// A1T2: rasterize_line
	auto delta = vb.fb_position.xy() - va.fb_position.xy();
	// TODO: Check out the block comment above this function for more information on how to fill in
	// this function!
	// The OpenGL specification section 3.5 may also come in handy.
	// Check for a horizontal line
	int x = static_cast<int>(va.fb_position.x);
	int y = static_cast<int>(va.fb_position.y);
    if (delta.y == 0.0f) {
        // Handle horizontal line here
        int start_x = static_cast<int>(std::min(va.fb_position.x, vb.fb_position.x));
        int end_x   = static_cast<int>(std::max(va.fb_position.x, vb.fb_position.x));
        for (float x = start_x + 0.5f; x <= end_x + 0.5; x += 1.0f) {
            Fragment frag;
            frag.fb_position.x = x;
			frag.fb_position.y = y + 0.5f;
            frag.fb_position.z = va.fb_position.z + (vb.fb_position.z - va.fb_position.z) * ((x - va.fb_position.x) / delta.x);
            frag.attributes = va.attributes;
            frag.derivatives.fill(Vec2(0.0f, 0.0f));
            emit_fragment(frag);
        }
    } else if (delta.x == 0.0f) {
		int start_y = static_cast<int>(std::min(va.fb_position.y, vb.fb_position.y));
        int end_y   = static_cast<int>(std::max(va.fb_position.y, vb.fb_position.y));
        for (float y = start_y + 0.5f; y <= end_y + 0.5; y+= 1.0f) {
            Fragment frag;
            frag.fb_position.y = y;
			frag.fb_position.x = x + 0.5f;
            frag.fb_position.z = va.fb_position.z + (vb.fb_position.z - va.fb_position.z) * ((y - va.fb_position.y) / delta.y);
            frag.attributes = va.attributes;
            frag.derivatives.fill(Vec2(0.0f, 0.0f));
            emit_fragment(frag);
        }
	} else {
		auto emit_frag = [&](int x, int y) {
			Fragment frag;
			frag.fb_position.x = static_cast<float>(x) + 0.5f;
			frag.fb_position.y = static_cast<float>(y) + 0.5f;
			frag.fb_position.z = va.fb_position.z + 
								(vb.fb_position.z - va.fb_position.z) * 
								((static_cast<float>(y) - va.fb_position.y) / (static_cast<float>(x) - va.fb_position.x));
			frag.attributes = va.attributes;
			frag.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(frag);
		};
		// bool steep = dy > dx;
		int x0 = static_cast<int>(va.fb_position.x);
		int y0 = static_cast<int>(va.fb_position.y);
		int x1 = static_cast<int>(vb.fb_position.x);
		int y1 = static_cast<int>(vb.fb_position.y);
		bool steep = abs(y1 - y0) > abs(x1 - x0);
		if (steep) {
			std::swap(x0, y0);
			std::swap(x1, y1);
		}

		if (x0 > x1) {
			std::swap(x0, x1);
			std::swap(y0, y1);
		}
		float dy = abs(y1 - y0);
		float dx = x1 - x0;
		float error = 0.0f;
		float delta_err = dy/ dx;
		int ystep;
		int y = y0;
		if (y0 < y1) {
			ystep = 1;
		} else {
			ystep = -1;
		}
		for (int x = x0; x < x1; x++) {
			if(steep) {
				emit_frag(y, x);
			} else {
				emit_frag(x, y);
			}
			error += delta_err;
			if (fabsf(error) >= 0.5) {
				y += ystep;
				error -= 1.0f;
			}
		}
	}
}

/*
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  	(x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Smooth: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 * 	The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 * 
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
inline float edgeFunction(const Vec2 &a, const Vec2 &b, const Vec2 &c) {
	return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]);
}

template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {
	// NOTE: it is okay to restructure this function to allow these tasks to use the
	//  same code paths. Be aware, however, that all of them need to remain working!
	//  (e.g., if you break Flat while implementing Correct, you won't get points
	//   for Flat.)
	int min_x = static_cast<int>(std::min({va.fb_position.x, vb.fb_position.x, vc.fb_position.x}));
	int max_x = static_cast<int>(std::max({va.fb_position.x, vb.fb_position.x, vc.fb_position.x}));
	int min_y = static_cast<int>(std::min({va.fb_position.y, vb.fb_position.y, vc.fb_position.y}));
	int max_y = static_cast<int>(std::max({va.fb_position.y, vb.fb_position.y, vc.fb_position.y}));
	float area = edgeFunction(va.fb_position.xy(), vb.fb_position.xy(), vc.fb_position.xy());
	// Compute derivatives
    Vec2 d_dx(1.0f, 0.0f);
    Vec2 d_dy(0.0f, 1.0f);
    float h = 1.0f; // step size, adjust as needed
	if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
		// A1T3: flat triangles
		// TODO: rasterize triangle (see block comment above this function).
		// As a placeholder, here's code that draws some lines:
		//(remove this and replace it with a real solution)
		// std::cout << "triangle area: " << area << std::endl;
		for(int x = min_x; x <= max_x; x++) {
			for(int y = min_y; y <= max_y; y++) {
				Vec2 pos = Vec2(x + 0.5f, y + 0.5f);
				float w_c = edgeFunction(va.fb_position.xy(), vb.fb_position.xy(), pos);
				float w_a = edgeFunction(vb.fb_position.xy(), vc.fb_position.xy(), pos);
				float w_b = edgeFunction(vc.fb_position.xy(), va.fb_position.xy(), pos);
				w_c /= area;
				w_a /= area;
				w_b /= area;
				if (w_c >=0 && w_a >=0 && w_b >= 0) {
					Fragment frag;
					frag.fb_position.x = static_cast<float>(x)+ 0.5f;
					frag.fb_position.y = static_cast<float>(y)+ 0.5f;
					frag.fb_position.z = w_a * va.fb_position.z + w_b * vb.fb_position.z + w_c * vc.fb_position.z;
					frag.attributes = va.attributes;
					frag.derivatives.fill(Vec2(0.0f, 0.0f));
					emit_fragment(frag);
				}
			}
		}
		// Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(va, vb, emit_fragment);
		// Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(vb, vc, emit_fragment);
		// Pipeline<PrimitiveType::Lines, P, flags>::rasterize_line(vc, va, emit_fragment);
	} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) {
		// A1T5: screen-space smooth triangles
		// TODO: rasterize triangle (see block comment above this function).
		auto interpolate_attribute = [&va, &vb, &vc, &area](float a, float b, float c, const Vec2 &pos) -> float {
			float w_c = edgeFunction(va.fb_position.xy(), vb.fb_position.xy(), pos);
			float w_a = edgeFunction(vb.fb_position.xy(), vc.fb_position.xy(), pos);
			float w_b = edgeFunction(vc.fb_position.xy(), va.fb_position.xy(), pos);
			w_c /= area;
			w_a /= area;
			w_b /= area;
			return w_a * a + w_b * b + w_c * c;
		};
		for(int x = min_x; x <= max_x; x++) {
			for(int y = min_y; y <= max_y; y++) {
				Vec2 pos = Vec2(x + 0.5f, y + 0.5f);
				float w_c = edgeFunction(va.fb_position.xy(), vb.fb_position.xy(), pos);
				float w_a = edgeFunction(vb.fb_position.xy(), vc.fb_position.xy(), pos);
				float w_b = edgeFunction(vc.fb_position.xy(), va.fb_position.xy(), pos);
				w_c /= area;
				w_a /= area;
				w_b /= area;
				if (w_c >=0 && w_a >=0 && w_b >= 0) {
					Fragment frag;
					frag.fb_position.x = static_cast<float>(x) + 0.5f;
					frag.fb_position.y = static_cast<float>(y) + 0.5f;
					frag.fb_position.z = w_a * va.fb_position.z + w_b * vb.fb_position.z + w_c * vc.fb_position.z;
					// frag.attributes = va.attributes;
					for(size_t i = 0; i < frag.attributes.size(); i++) {
						frag.attributes[i] = w_a * va.attributes[i] + w_b * vb.attributes[i] + w_c *  vc.attributes[i];
					}
					// frag.derivatives.fill(Vec2(0.0f, 0.0f));
					for(int i = 0; i < 2; i++) {
						float f_x_plus_h  = interpolate_attribute(va.attributes[i], vb.attributes[i], vc.attributes[i], pos + d_dx*h);
						float f_x_minus_h = interpolate_attribute(va.attributes[i], vb.attributes[i], vc.attributes[i], pos - d_dx*h);

						float f_y_plus_h  = interpolate_attribute(va.attributes[i], vb.attributes[i], vc.attributes[i], pos + d_dy*h);
						float f_y_minus_h = interpolate_attribute(va.attributes[i], vb.attributes[i], vc.attributes[i], pos - d_dy*h);
						float df_dx = (f_x_plus_h - f_x_minus_h) / 2 * h;
						float df_dy = (f_y_plus_h - f_y_minus_h) / 2 * h;
						frag.derivatives[i] = Vec2(df_dx, df_dy);
					}
					// frag.derivatives.fill(Vec2(0.0f, 0.0f));
					emit_fragment(frag);
				}
			}
		}
		// As a placeholder, here's code that calls the Flat interpolation version of the function:
		//(remove this and replace it with a real solution)
		// Pipeline<PrimitiveType::Lines, P, (flags & ~PipelineMask_Interp) | Pipeline_Interp_Flat>::rasterize_triangle(va, vb, vc, emit_fragment);
	} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) {
		// A1T5: perspective correct triangles
		// TODO: rasterize triangle (block comment above this function).

		// As a placeholder, here's code that calls the Screen-space interpolation function:
		//(remove this and replace it with a real solution)
		// Pipeline<PrimitiveType::Lines, P, (flags & ~PipelineMask_Interp) | Pipeline_Interp_Smooth>::rasterize_triangle(va, vb, vc, emit_fragment);
		auto interpolate_attribute = [&va, &vb, &vc, &area](const Vec2 &pos, int i) -> float {
			float w_c = edgeFunction(va.fb_position.xy(), vb.fb_position.xy(), pos);
			float w_a = edgeFunction(vb.fb_position.xy(), vc.fb_position.xy(), pos);
			float w_b = edgeFunction(vc.fb_position.xy(), va.fb_position.xy(), pos);
			w_c /= area;
			w_a /= area;
			w_b /= area;
			float attr_inv_w = w_a * va.attributes[i] * va.inv_w + w_b * vb.attributes[i] * vb.inv_w + w_c *  vc.attributes[i] * vc.inv_w;
			float interplot_inv_w = va.inv_w * w_a + vb.inv_w * w_b + vc.inv_w * w_c;
			return attr_inv_w / interplot_inv_w;
		};
		for(int x = min_x; x <= max_x; x++) {
			for(int y = min_y; y <= max_y; y++) {
				Vec2 pos = Vec2(x + 0.5f, y + 0.5f);
				float w_c = edgeFunction(va.fb_position.xy(), vb.fb_position.xy(), pos);
				float w_a = edgeFunction(vb.fb_position.xy(), vc.fb_position.xy(), pos);
				float w_b = edgeFunction(vc.fb_position.xy(), va.fb_position.xy(), pos);
				w_c /= area;
				w_a /= area;
				w_b /= area;
				if (w_c >=0 && w_a >=0 && w_b >= 0) {
					Fragment frag;
					frag.fb_position.x = static_cast<float>(x) + 0.5f;
					frag.fb_position.y = static_cast<float>(y) + 0.5f;
					frag.fb_position.z = w_a * va.fb_position.z + w_b * vb.fb_position.z + w_c * vc.fb_position.z;
					// frag.attributes = va.attributes;
					for(size_t i = 0; i < frag.attributes.size(); i++) {
						float attr_inv_w = w_a * va.attributes[i] * va.inv_w + w_b * vb.attributes[i] * vb.inv_w + w_c *  vc.attributes[i] * vc.inv_w;
						float interplot_inv_w = va.inv_w * w_a + vb.inv_w * w_b + vc.inv_w * w_c;
						frag.attributes[i] = attr_inv_w / interplot_inv_w;
					}
					for(int i = 0; i < 2; i++) {
						float f_x_plus_h  = interpolate_attribute(pos + d_dx*h, i);
						float f_x_minus_h = interpolate_attribute(pos - d_dx*h, i);

						float f_y_plus_h  = interpolate_attribute(pos + d_dy*h, i);
						float f_y_minus_h = interpolate_attribute(pos - d_dy*h, i);
						float df_dx = (f_x_plus_h - f_x_minus_h) / 2 * h;
						float df_dy = (f_y_plus_h - f_y_minus_h) / 2 * h;
						frag.derivatives[i] = Vec2(df_dx, df_dy);
					}
					emit_fragment(frag);
				}
			}
		}
	}
}

//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;