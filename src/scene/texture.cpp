
#include "texture.h"

#include <iostream>

namespace Textures {


Spectrum sample_nearest(HDR_Image const &image, Vec2 uv) {
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	//the pixel with the nearest center is the pixel that contains (x,y):
	int32_t ix = int32_t(std::floor(x));
	int32_t iy = int32_t(std::floor(y));

	//texture coordinates of (1,1) map to (w,h), and need to be reduced:
	ix = std::min(ix, int32_t(image.w) - 1);
	iy = std::min(iy, int32_t(image.h) - 1);

	return image.at(ix, iy);
}

Spectrum sample_bilinear(HDR_Image const &image, Vec2 uv) {
	// A1T6: sample_bilinear
	//TODO: implement bilinear sampling strategy on texture 'image'
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);
	int i = std::floor(x - 0.5);
	int j = std::floor(y - 0.5);
	float s = x - (i + 0.5);
	float t = y - (j + 0.5);
	int ix = int32_t(std::floor(i)) % image.w;
	int iy = int32_t(std::floor(j)) % image.h;

	int ix_2 = (ix + 1) % image.w;
	int iy_2 = (iy + 1) % image.h;

	// Sample the four corner points
    Spectrum c00 = image.at(ix, iy);
    Spectrum c10 = image.at(ix_2, iy);
    Spectrum c01 = image.at(ix, iy_2);
    Spectrum c11 = image.at(ix_2, iy_2);
    // Interpolate between the corner points
    Spectrum c0 = c00 * (1.0f - s) + c10 * s;
    Spectrum c1 = c01 * (1.0f - s) + c11 * s;
    Spectrum c = c0 * (1.0f - t) + c1 * t;
    return c;
}


Spectrum sample_trilinear(HDR_Image const &base, std::vector< HDR_Image > const &levels, Vec2 uv, float lod) {
	// A1T6: sample_trilinear
	//TODO: implement trilinear sampling strategy on using mip-map 'levels'

	// float clamped_lod = std::clamp(lod, 0.0f, static_cast<float>(levels.size()));

	// int lod_int = std::floor(clamped_lod);
	// float w = clamped_lod - lod_int;
	// lod_int -=1;
	// Spectrum lower_level;
	// Spectrum upper_level;
	// if (lod_int == -1) {
	// 	lower_level = sample_bilinear(base, uv);
	// } else  {
	// 	lower_level = sample_bilinear(levels[lod_int], uv);
	// }
	// if (lod_int == levels.size() - 1){
	// 	upper_level = sample_bilinear(levels[lod_int], uv);
	// } else {
	// 	upper_level = sample_bilinear(levels[lod_int+1], uv);
	// }
	// Spectrum c = lower_level * (1.0 - w) + upper_level * w;

	return sample_nearest(base, uv); //placeholder so image doesn't look blank
}

/*
 * generate_mipmap- generate mipmap levels from a base image.
 *  base: the base image
 *  levels: pointer to vector of levels to fill (must not be null)
 *
 * generates a stack of levels [1,n] of sizes w_i, h_i, where:
 *   w_i = max(1, floor(w_{i-1})/2)
 *   h_i = max(1, floor(h_{i-1})/2)
 *  with:
 *   w_0 = base.w
 *   h_0 = base.h
 *  and n is the smalles n such that w_n = h_n = 1
 *
 * each level should be calculated by downsampling a blurred version
 * of the previous level to remove high-frequency detail.
 *
 */
void generate_mipmap(HDR_Image const &base, std::vector< HDR_Image > *levels_) {
	assert(levels_);
	auto &levels = *levels_;


	{ // allocate sublevels sufficient to scale base image all the way to 1x1:
		int32_t num_levels = static_cast<int32_t>(std::log2(std::max(base.w, base.h)));
		assert(num_levels >= 0);

		levels.clear();
		levels.reserve(num_levels);

		uint32_t width = base.w;
		uint32_t height = base.h;
		for (int32_t i = 0; i < num_levels; ++i) {
			assert(!(width == 1 && height == 1)); //would have stopped before this if num_levels was computed correctly

			width = std::max(1u, width / 2u);
			height = std::max(1u, height / 2u);

			levels.emplace_back(width, height);
		}
		assert(width == 1 && height == 1);
		assert(levels.size() == uint32_t(num_levels));
	}

	//now fill in the levels using a helper:
	//downsample:
	// fill in dst to represent the low-frequency component of src
	auto downsample = [](HDR_Image const &src, HDR_Image &dst) {
		//dst is half the size of src in each dimension:
		assert(std::max(1u, src.w / 2u) == dst.w);
		assert(std::max(1u, src.h / 2u) == dst.h);

		// A1T6: generate
		//TODO: Write code to fill the levels of the mipmap hierarchy by downsampling
		for(uint32_t y = 0; y < dst.h; y++) {
			for(uint32_t x = 0; x < dst.w; x++) {
				float src_x = x * 2.0f;
				float src_y = y * 2.0f;

				Spectrum c00 = src.at(std::floor(src_x), std::floor(src_y));
				Spectrum c10 = src.at(std::floor(src_x) + 1, std::floor(src_y));
				Spectrum c01 = src.at(std::floor(src_x), std::floor(src_y) + 1);
				Spectrum c11 = src.at(std::floor(src_x) + 1, std::floor(src_y) + 1);
				Spectrum c = (c00 + c10 + c01 + c11) / 4;

                dst.at(x, y) =  c;
			}
		}
		//Be aware that the alignment of the samples in dst and src will be different depending on whether the image is even or odd.

	};

	std::cout << "Regenerating mipmap (" << levels.size() << " levels): [" << base.w << "x" << base.h << "]";
	std::cout.flush();
	for (uint32_t i = 0; i < levels.size(); ++i) {
		HDR_Image const &src = (i == 0 ? base : levels[i-1]);
		HDR_Image &dst = levels[i];
		std::cout << " -> [" << dst.w << "x" << dst.h << "]"; std::cout.flush();

		downsample(src, dst);
	}
	std::cout << std::endl;
	
}

Image::Image(Sampler sampler_, HDR_Image const &image_) {
	sampler = sampler_;
	image = image_.copy();
	update_mipmap();
}

Spectrum Image::evaluate(Vec2 uv, float lod) const {
	if (sampler == Sampler::nearest) {
		return sample_nearest(image, uv);
	} else if (sampler == Sampler::bilinear) {
		return sample_bilinear(image, uv);
	} else {
		return sample_trilinear(image, levels, uv, lod);
	}
}

void Image::update_mipmap() {
	if (sampler == Sampler::trilinear) {
		generate_mipmap(image, &levels);
	} else {
		levels.clear();
	}
}

GL::Tex2D Image::to_gl() const {
	return image.to_gl(1.0f);
}

void Image::make_valid() {
	update_mipmap();
}

Spectrum Constant::evaluate(Vec2 uv, float lod) const {
	return color * scale;
}

} // namespace Textures
bool operator!=(const Textures::Constant& a, const Textures::Constant& b) {
	return a.color != b.color || a.scale != b.scale;
}

bool operator!=(const Textures::Image& a, const Textures::Image& b) {
	return a.image != b.image;
}

bool operator!=(const Texture& a, const Texture& b) {
	if (a.texture.index() != b.texture.index()) return false;
	return std::visit(
		[&](const auto& data) { return data != std::get<std::decay_t<decltype(data)>>(b.texture); },
		a.texture);
}
