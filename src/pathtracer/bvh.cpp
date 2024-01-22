
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"

#include <stack>
#include <queue>
#include <set>
#include <iostream>
namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

template<typename Primitive>
void BVH<Primitive>::sah_split(size_t node, size_t max_leaf_size) {
	if (nodes[node].size <= max_leaf_size) {
		// nodes[node].l = nodes[node].r = -1;
		return;
	}
	auto cost = FLT_MAX;
	const int n_buckets = 16;
	BBox left_box, right_box;
	int axis = -1;
	int right_count = -1;
    std::vector<SAHBucketData> buckets(n_buckets);
	Node& cur_node = nodes[node];
	for (auto dim: {0, 1, 2}) {
		for(size_t i = cur_node.start; i < cur_node.start + cur_node.size; i++) {
			BBox box = primitives[i].bbox();
			Vec3 centor = box.center();
			const auto ratio = (centor[dim] - cur_node.bbox.min[dim]) / cur_node.bbox.diagonal()[dim];
			auto index = int(n_buckets * ratio);

			if (index == n_buckets) index = n_buckets - 1;

			++buckets[index].num_prims;
			buckets[index].bb.enclose(box);
		}

		// Calculate cumulative distribution of bounding volumes [left to right]
		std::vector<BBox> cumulative_buckets(n_buckets);
		cumulative_buckets[0] = buckets[0].bb;
		for (auto i = 1; i < n_buckets; ++i)
		{
			cumulative_buckets[i] = cumulative_buckets[i - 1];
			cumulative_buckets[i].enclose(buckets[i].bb);
		}

		BBox b1;
		int count1 = 0;

		// Calculate cost of splitting the bounding volumes [right to left]
		for (auto i = n_buckets - 1; i >= 1; --i)
		{
			b1.enclose(buckets[i].bb);
			count1 += buckets[i].num_prims;

			auto count0 = cur_node.size - count1;
			auto sum = count0 * cumulative_buckets[i - 1].surface_area() + count1 * b1.surface_area();

			// Surface area cost weighted by number of elements in a bucket
			auto current_cost = .125f + sum / cur_node.bbox.surface_area();

			if (cost > current_cost)
			{
				left_box = cumulative_buckets[i - 1];
				right_box = b1;
				cost = current_cost;
				right_count = count1;
				axis = dim;
			}
		}

		std::fill(buckets.begin(), buckets.end(), SAHBucketData());
	}
	auto begin = primitives.begin() + cur_node.start;
	auto end   = primitives.begin() + cur_node.start + cur_node.size;
	auto mid   = primitives.begin() + cur_node.start + cur_node.size - right_count;

	auto left_node  = new_node(left_box, cur_node.start, cur_node.size- right_count);
	auto right_node = new_node(right_box, cur_node.start + cur_node.size - right_count, right_count);
	nodes[node].l = left_node;
	nodes[node].r = right_node;

	std::nth_element(begin, mid, end, [axis](Primitive& l , Primitive& r){
				return l.bbox().center()[axis] <  r.bbox().center()[axis];
	});

	sah_split(left_node, max_leaf_size);
	sah_split(right_node, max_leaf_size);
}

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh
	// std::cout << "now in build bvh" << std::endl;
	// Keep these
    nodes.clear();
    primitives = std::move(prims);
	// std::cout << "init primitives" << std::endl;
	// std::cout << "size: " << primitives.size() << std::endl;

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.

	//TODO
	size_t r = new_node({}, 0, primitives.size());
	for (auto& primitive: primitives) {
		nodes[r].bbox.enclose(primitive.bbox());
	}
	sah_split(r, max_leaf_size);
	
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH

    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
    Trace ret;
	if(primitives.size() != 0) {
		find_closest_hit(ray, 0, ret);
	}
    return ret;
}

template<typename Primitive> 
void BVH<Primitive>::find_closest_hit(const Ray& ray,  size_t rt, Trace& closest) const {
	
	const auto& node = nodes[rt];
	if (node.is_leaf()) {
		for(size_t i = node.start; i < node.start + node.size; i++) {
			closest = Trace::min(closest, primitives[i].hit(ray));
		}
	} else {
		Vec2 ray_dist_bounds_L = ray.dist_bounds;
		auto distL = nodes[node.l].bbox.hit(ray, ray_dist_bounds_L) ? ray_dist_bounds_L.x: FLT_MAX;
		Vec2 ray_dist_bounds_R = ray.dist_bounds;
		auto distR = nodes[node.r].bbox.hit(ray, ray_dist_bounds_R) ? ray_dist_bounds_R.x: FLT_MAX;
		auto first  = node.l;
		auto second = node.r;
		if (distL > distR) {
			std::swap(distL, distR);
			std::swap(first, second);
		}
		if(distL == FLT_MAX) {
			return;
		} 
		find_closest_hit(ray, first, closest);
		if (!closest.hit || distR < closest.distance) {
			find_closest_hit(ray, second, closest);
		}
	}
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
