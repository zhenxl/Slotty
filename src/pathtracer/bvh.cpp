
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
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	//A3T3 - build a bvh
	// std::cout << "now in build bvh" << std::endl;
	// Keep these
    nodes.clear();
    primitives = std::move(prims);
	// std::cout << "init primitives" << std::endl;

	std::vector<size_t> indices(primitives.size());
	// std::cout << "size: " << primitives.size() << std::endl;

    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration.

	//TODO
	std::queue<size_t> queue;
	size_t r = new_node({}, 0, primitives.size());
	queue.push(r);
	// std::cout << "init queue" << std::endl;
	while(!queue.empty()) {
		// std::cout << "not queue empty " << queue.size() << std::endl;
		size_t root_idx = queue.front();
		Node& root = nodes[root_idx];
		queue.pop();
		if (root.size <= max_leaf_size) {
			for(size_t i = root.start; i <root.start + root.size; i++) {
				nodes[root_idx].bbox.enclose(primitives[i].bbox());
			}
		} else {

			// std::cout <<"now create inter node" << std::endl;
			float best_cost = FLT_MAX;
			std::set<size_t> best_left_fit_set;
			const auto p_begin = static_cast<ptrdiff_t> (root.start);
			const auto p_end = static_cast<ptrdiff_t> (root.start + root.size);
			std::iota(indices.begin() + p_begin, indices.begin() + p_end, p_begin);
			// std::cout <<"before find partition" << std::endl;
			for(size_t dim = 0; dim < 3; dim++) {
				size_t bucket_num = 12;
				if (root.size < bucket_num) {
					bucket_num = root.size;
				}
				std::vector<SAHBucketData> buckets(bucket_num);
				std::sort(indices.begin() + p_begin, indices.begin() + p_end,
                [this, dim](int l, int r) {
                  return primitives[l].bbox().center()[dim] < primitives[r].bbox().center()[dim];
                });
				size_t inter_cnt = root.size / bucket_num; 
				for(size_t i = 0; i < bucket_num; i++) {
					for(size_t j = root.start + i* inter_cnt; j <  root.start + (i+1)*inter_cnt; j++) {
						buckets[i].num_prims++;
						buckets[i].bb.enclose(primitives[indices[j]].bbox());
					}
				}
				for(size_t j = root.start + (bucket_num ) * inter_cnt; j < root.start + root.size; j++) {
					buckets[bucket_num - 1].num_prims++;
					buckets[bucket_num - 1].bb.enclose(primitives[j].bbox());
				}

				float cost[bucket_num -1];
				for(size_t i = 1; i < bucket_num; i++) {
					BBox b0, b1;
					int count0 = 0, count1 = 0;
					for(size_t j = 0; j < i; j++) {
						b0.enclose(buckets[i].bb);
						count0 +=1;
					}
					for(size_t j = i; j < bucket_num; j++) {
						b1.enclose(buckets[j].bb);
						count1 += 1;
					}
					cost[i-1] = count0 * b0.surface_area() + count1 * b1.surface_area();
				}

				float min_cost = cost[0];
				size_t min_cost_bucket = 1;
				for(size_t i = 1; i < bucket_num - 1; i++) {
					if (cost[i] < min_cost) {
						min_cost = cost[i];
						min_cost_bucket = i+1;
					}
				}

				if(min_cost < best_cost) {
					best_cost = min_cost;
					std::set<size_t> best_left_set{indices.begin() + p_begin, indices.begin() + p_begin + min_cost_bucket * inter_cnt};
					best_left_fit_set = std::move(best_left_set);
				}

			}
			// std::cout << "get best partition" << std::endl;

			//now apply best partition
			{
				size_t left_cursor = p_begin;
				for(auto index: best_left_fit_set) {
					if (left_cursor != index) {
						std::swap(primitives[left_cursor], primitives[index]);
					}
					left_cursor ++;
				}

				size_t left_size = best_left_fit_set.size();
				BBox left_box, right_box;
				for(size_t i = root.start; i < root.start + left_size; i++) {
					left_box.enclose(primitives[i].bbox());
				}
				size_t right_start = root.start + left_size;
				size_t right_size  = root.size - left_size;
				// std::cout << "left size: " << left_size << std::endl;
				// std::cout << "right size: " << right_size << std::endl;

				for(size_t i = right_start; i < right_start + right_size; i++) {
					right_box.enclose(primitives[i].bbox());
				}

				size_t left_node = new_node(left_box, root.start, left_size);
				size_t right_node = new_node(right_box, right_start, right_size);

				nodes[root_idx].bbox.enclose(left_box);
				nodes[root_idx].bbox.enclose(right_box);

				nodes[root_idx].l = left_node;
				nodes[root_idx].r = right_node;

				queue.push(left_node);
			    queue.push(right_node);
			}
		}
	}
	// std::cout << "build finished" << std::endl;

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
