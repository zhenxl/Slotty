
#include "transform.h"

Mat4 Transform::local_to_parent() const {
	return Mat4::translate(translation) * rotation.to_mat() * Mat4::scale(scale);
}

Mat4 Transform::parent_to_local() const {
	return Mat4::scale(1.0f / scale) * rotation.inverse().to_mat() * Mat4::translate(-translation);
}

Mat4 Transform::local_to_world() const {
	// A1T1: local_to_world
	//don't use Mat4::inverse() in your code.
	const auto l2p = local_to_parent();
	if (parent.expired()) {
		return l2p;
	} else {
		auto parent_transform = parent.lock();
		Mat4 parent_2_world = parent_transform->local_to_world();
		return  parent_2_world * l2p;
	}
}

Mat4 Transform::world_to_local() const {
	// A1T1: world_to_local
	//don't use Mat4::inverse() in your code.
	const auto p2l = parent_to_local();
	if (parent.expired()) {
		return p2l;
	} else {
		auto parent_transform = parent.lock();
		Mat4 w2p = parent_transform->world_to_local();
		return p2l * w2p;
	}
}

bool operator!=(const Transform& a, const Transform& b) {
	return a.parent.lock() != b.parent.lock() || a.translation != b.translation ||
	       a.rotation != b.rotation || a.scale != b.scale;
}
