
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
    FaceRef f_not_used = emplace_face();
    HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
    erase_face(f_not_used);
    erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge
	
	// (void)e; //this line avoids 'unused parameter' warnings. You can delete it as you fill in the function.

	// if (e->isBoundary()) return e->halfedge()->vertex();
	auto v = bisect_edge(e);

	if (v.has_value()) {
		if (e->on_boundary()) {
			VertexRef v0 = v.value();
			HalfedgeRef h0 = v0->halfedge;
			if (h0->face == faces.end()) {
				h0 = h0->twin;
			}
			HalfedgeRef h1 = h0->next;
			HalfedgeRef h2 = h1->next;
			HalfedgeRef h4 = h0->twin->next->twin;
			
			// VertexRef v0 = h0->vertex;
			VertexRef v4 = h2->vertex;

			FaceRef f0 = h2->face;

			FaceRef f2 = emplace_face();
			EdgeRef e1 = emplace_edge();

			HalfedgeRef h10 = emplace_halfedge();
			HalfedgeRef h11 = emplace_halfedge();

			h0->set_tnvef(h0->twin, h1, v0, h0->edge, f2);
			h1->set_tnvef(h1->twin, h10, h1->vertex, h1->edge, f2);
			h10->set_tnvef(h11, h0, v4, e1, f2);

			f2->halfedge = h0;
			e1->halfedge = h10;

			h4->next = h11;
			h11->set_tnvef(h10, h2, v0, e1, f0);
			f0->halfedge = h11;
			return v0;
		} else {
			VertexRef v0 = v.value();
			HalfedgeRef h0 = v0->halfedge;
			HalfedgeRef h1 = h0->next;
			auto h2 = h1->next;

			auto h5 = h0->twin;
			auto h6 = h5->next;
			auto h4 = h6->twin;

			auto h7 = h6->next;
			auto h8 = h7->next;

			VertexRef v1 = h5->vertex;
			VertexRef v2 = h7->vertex;

			VertexRef v3 = h8->vertex;
			VertexRef v4 = h2->vertex;

			FaceRef f0 = h4->face;
			FaceRef f1 = h8->face;

			EdgeRef e1 = emplace_edge();
			EdgeRef e2 = emplace_edge();

			FaceRef f2 = emplace_face();
			FaceRef f3 = emplace_face();

			HalfedgeRef h10 = emplace_halfedge();
			HalfedgeRef h11 = emplace_halfedge();

			HalfedgeRef h12 = emplace_halfedge();
			HalfedgeRef h13 = emplace_halfedge();

			h4->next = h11;
			h11->set_tnvef(h10, h2, v0, e1, f0);

			h10->set_tnvef(h11, h0, v4, e1, f2);
			h0->set_tnvef(h5, h1, v0, h0->edge, f2);
			h1->set_tnvef(h1->twin, h10, v1, h1->edge, f2);
			e1->halfedge = h10;
			f0->halfedge = h4;
			f2->halfedge = h0;

			h5->next = h12;
			h12->set_tnvef(h13, h8, v0, e2, f1);

			h13->set_tnvef(h12, h6, v3, e2, f3);
			h6->set_tnvef(h4, h7, v0, h6->edge, f3);
			h7->set_tnvef(h7->twin, h13, v2, h7->edge, f3);

			e2->halfedge = h13;
			f1->halfedge = h5;
			f3->halfedge = h6;

			return v0;
		}
	} else {
		 return std::nullopt;
	}
}



/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	
	(void)f;
    return std::nullopt;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in extrude_helper (A2L4h)

	HalfedgeRef h0 = f->halfedge;
	HalfedgeRef h1 = h0->next;
	HalfedgeRef h2 = h1->next;
	HalfedgeRef h3 = h2->next;
	VertexRef v0 = h0->vertex;
	VertexRef v1 = h1->vertex;
	VertexRef v2 = h2->vertex;
	VertexRef v3 = h3->vertex;

	VertexRef v4 = emplace_vertex();
	VertexRef v5 = emplace_vertex();
	VertexRef v6 = emplace_vertex();
	VertexRef v7 = emplace_vertex();

	// give each vertex position
	v4->position = v0->position;
	v5->position = v1->position;
	v6->position = v2->position;
	v7->position = v3->position;

	EdgeRef e1 = emplace_edge();
	EdgeRef e2 = emplace_edge();
	EdgeRef e3 = emplace_edge();
	EdgeRef e4 = emplace_edge();
	EdgeRef e5 = emplace_edge();
	EdgeRef e6 = emplace_edge();
	EdgeRef e7 = emplace_edge();
	EdgeRef e8 = emplace_edge();


	HalfedgeRef h4 = emplace_halfedge();
	HalfedgeRef h5 = emplace_halfedge();
	HalfedgeRef h6 = emplace_halfedge();
	HalfedgeRef h7 = emplace_halfedge();

	HalfedgeRef h8  = emplace_halfedge();
	HalfedgeRef h9  = emplace_halfedge();
	HalfedgeRef h10 = emplace_halfedge();
	HalfedgeRef h11 = emplace_halfedge();

	HalfedgeRef h12 = emplace_halfedge();
	HalfedgeRef h13 = emplace_halfedge();
	HalfedgeRef h14 = emplace_halfedge();
	HalfedgeRef h15 = emplace_halfedge();
	HalfedgeRef h16 = emplace_halfedge();
	HalfedgeRef h17 = emplace_halfedge();
	HalfedgeRef h18 = emplace_halfedge();
	HalfedgeRef h19 = emplace_halfedge();

	// FaceRef not_used_face = f;
	// FaceRef f1 = emplace_face();
	FaceRef f2 = emplace_face();
	FaceRef f3 = emplace_face();
	FaceRef f4 = emplace_face();
	FaceRef f5 = emplace_face();

	//reassign connectivity for f1
	h4->set_tnvef(h8, h5, v4, e1, f);
	h5->set_tnvef(h11, h6, v5, e2, f);
	h6->set_tnvef(h10, h7, v6, e3, f);
	h7->set_tnvef(h9, h4, v7, e4, f);
	e1->halfedge = h4;
	e2->halfedge = h5;
	e3->halfedge = h6;
	e4->halfedge = h7;
	f->halfedge = h4;

	v4->halfedge = h4;
	v5->halfedge = h5;
	v6->halfedge = h6;
	v7->halfedge = h7;

	//reassign connectivity for f2
	h8->set_tnvef(h4, h12, v5, e1, f2);
	h12->set_tnvef(h13, h0, v4, e5, f2);
	h0->set_tnvef(h0->twin, h14, v0, h0->edge, f2);
	h14->set_tnvef(h15, h8, v1, e6, f2);
	e5->halfedge = h12;
	e6->halfedge = h14;
	f2->halfedge = h8;

	//reassign connectivity for f3
	h1->set_tnvef(h1->twin, h16, v1, h1->edge, f3);
	h16->set_tnvef(h17, h11, v2, e7, f3);
	h11->set_tnvef(h5, h15, v6, e2, f3);
	h15->set_tnvef(h14, h1, v5, e6, f3);
	e7->halfedge = h16;
	f3->halfedge = h15;

	//reassign connectivity for f4
	h2->set_tnvef(h2->twin, h18, v2, h2->edge, f4);
	h18->set_tnvef(h19, h10, v3, e8, f4);
	h10->set_tnvef(h6, h17, v7, e3, f4);
	h17->set_tnvef(h16, h2, v6, e7, f4);
	e8->halfedge = h18;
	f4->halfedge = h2;

	//reassign connectivity for f5
	h3->set_tnvef(h3->twin, h13, v3, h3->edge, f5);
	h13->set_tnvef(h12, h9, v0, e5, f5);
	h9->set_tnvef(h7, h19, v4, e4, f5);
	h19->set_tnvef(h18, h3, v7, e8, f5);
	f5->halfedge = h3;

	// erase_face(not_used_face);

    return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge
	if (e->on_boundary()) {
		return std::nullopt;
	}
	
	HalfedgeRef h0 = e->halfedge;
	HalfedgeRef h1 = h0->twin;

	HalfedgeRef h2 = h0->next;
	HalfedgeRef h3 = h2->twin;

	HalfedgeRef h4 = h2->next;
	// HalfedgeRef h5 = h4->twin;

	HalfedgeRef h6 = h0->before();
	// HalfedgeRef h7 = h6->twin;

	HalfedgeRef h8 = h1->next;
	HalfedgeRef h9 = h8->twin;

	HalfedgeRef h10 = h8->next;
	// HalfedgeRef h11 = h10->next;

	HalfedgeRef h12 = h1->before();
	// HalfedgeRef h13 = h12->twin;

	VertexRef v0 = h0->vertex;
	VertexRef v1 = h2->vertex;
	VertexRef v2 = h9->vertex;
	VertexRef v3 = h3->vertex;

	FaceRef f0  = h0->face;
	FaceRef f1  = h1->face;

	// phase3 re connect
	h6->next = h8;
	h8->next = h0;
	h0->next = h4;
	h8->face = f0;

	h12->next = h2;
	h2->next  = h1;
	h1->next  = h10;
	h2->face  = f1;

	h0->vertex = v2;
	h1->vertex = v3;

	if (v0 ->halfedge == h0) {
		v0->halfedge = h8;
	}

	if (v1->halfedge == h1) {
		v1->halfedge = h2;
	}


	// TODO This method should split the given edge and return an iterator to the newly inserted vertex.
	// TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

	return e;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex

    return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	
    return std::nullopt;
}

bool Halfedge_Mesh::isDirectConnected(VertexCRef v1, VertexCRef v2) {
	if (v1 == v2) return false; //they are same vertex
	HalfedgeCRef h1 = v1->halfedge;
	do {
		HalfedgeCRef h1_twin = h1->twin;
		VertexCRef v1_neibour = h1_twin->vertex;
		if (v1_neibour == v2) return true;
		h1 = h1_twin->next;
	} while(h1 != v1->halfedge);
	return false;
}

bool Halfedge_Mesh::willCollapseCauseNonManifold(EdgeRef e) {
	VertexRef v1 = e->halfedge->vertex;
	VertexRef v2 = e->halfedge->twin->vertex;
	if (v1->on_boundary() && v2->on_boundary() && !e->on_boundary()) {
		return false;
	}

	HalfedgeCRef h = v1->halfedge;
	do 
	{
		HalfedgeCRef h_twin = h->twin;
		VertexCRef v1_direct_neibour = h_twin->vertex;
		HalfedgeCRef hh = v1->halfedge;
		// in inner loop, we need find another v1 neibour
		do {
			HalfedgeCRef hh_twin = hh->twin;
			VertexCRef another_v1_neibour = hh_twin->vertex;
			if (isDirectConnected(v1_direct_neibour, another_v1_neibour) && isDirectConnected(another_v1_neibour, v2) && isDirectConnected(v1_direct_neibour, v2)) {
				return true;
			}
			hh = hh_twin->next;
		} while(hh != v1->halfedge);
		h = h_twin->next;
	} while(h != v1->halfedge);

	return false;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge
	if (willCollapseCauseNonManifold(e)) return std::nullopt;

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)
	//now consider this condition:
	// |  |
	// |  |
	// |--|
	// |  |
	// |  |
	HalfedgeRef h1 = e->halfedge;
	HalfedgeRef h2 = h1->next;
	HalfedgeRef h3 = h1->before();
	HalfedgeRef h4 = h1->twin;
	HalfedgeRef h5 = h4->next;
	HalfedgeRef h6 = h4->before();
	HalfedgeRef h7 = h2->twin;
	HalfedgeRef h8 = h6->twin;
	HalfedgeRef h9 = h5->twin;
	HalfedgeRef h10 = h3->twin;

	VertexRef v0 = h4->vertex;
	VertexRef v1 = h1->vertex;
	VertexRef v2 = h3->vertex;
	VertexRef v3 = h7->vertex;
	VertexRef v4 = h6->vertex;
	VertexRef v5 = h9->vertex;

	FaceRef f0 = h1->face;
	FaceRef f1 = h4->face;

	VertexRef vm = emplace_vertex();
	vm->position = (v0->position + v1->position) / 2.0f;
	interpolate_data({v0, v1}, vm);
	if (v2->id == v3->id) {
		//create new element
		//reassign
		EdgeRef h7_edge = h7->edge;
		h7->set_tnvef(h10, h7->next, v2, h7_edge, h7->face);
		interpolate_data({h1, h2}, h7);
		h10->set_tnvef(h7, h10->next, vm, h7_edge, h10->face);
		interpolate_data({h3, h1}, h10);
		v2->halfedge = h7;
		h7->edge->halfedge = h7;
		//erase useless 
		// erase_edge(h3->edge);
		// erase_edge(e);

		// erase_halfedge(h2);
		// erase_halfedge(h3);
		// erase_halfedge(h1);

		// erase_face(f0);
		// erase_vertex(v0);
		// erase_vertex(v1);

	} else {
		h3->set_tnvef(h10, h2, v2, h3->edge, f0);
		h2->set_tnvef(h7, h2->next, vm, h2->edge, f0);
		f0->halfedge = h3;
		h10->vertex = vm;

	}

	if(v5->id == v4->id) {
		EdgeRef h8_edge = h8->edge;
		h8->set_tnvef(h9, h8->next, vm, h8_edge, h8->face);
		interpolate_data({h6, h4}, h9);
		h9->set_tnvef(h8, h9->next, v5, h8_edge, h9->face);
		h8->edge->halfedge = h8;
		v5->halfedge = h9;
	} else {
		h6->set_tnvef(h8, h5, v4, h6->edge, f1);
		h5->set_tnvef(h9, h5->next, vm, h5->edge, f1);
		f1->halfedge = h6;
		h8->vertex = vm;
	}
	vm->halfedge = h10;
	HalfedgeRef hx = h7;
	HalfedgeRef hy = hx->next;
	while (hy != h8) {
		hy->vertex = vm;
		hy = hy->twin->next;
	}
	hx = h9;
	hy = hx->next;
	while (hy != h10) {
		hy->vertex = vm;
		hy = hy->twin->next;
	}

	// erase edge
	if (v2->id == v3->id) {
		erase_face(f0);
		erase_edge(h3->edge);
		erase_halfedge(h2);
		erase_halfedge(h3);
	}
	if( v4->id == v5->id) {
		erase_face(f1);
		erase_edge(h5->edge);
		erase_halfedge(h5);
		erase_halfedge(h6);
	}
	erase_vertex(v0);
	erase_vertex(v1);
	erase_halfedge(h1);
	erase_halfedge(h4);
	erase_edge(e);
	return vm;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)

    return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink,
	// offset by move
	// Initialize centroid to zero
	Vec3 centroid(0.0f, 0.0f, 0.0f);
	float count = 0.0f;

	// Loop over all vertices of the face to compute the centroid
	HalfedgeRef face_hg = face->halfedge;
	do {
		centroid += face_hg->vertex->position;
		count += 1.0f;
		face_hg = face_hg->next;
	} while(face_hg != face->halfedge);
	
	centroid /= count; // Divide by the total number of vertices to get the average (centroid)
	face_hg = face->halfedge;

	do {
		VertexRef v = face_hg->vertex;
		Vec3 newPosition = v->position;
		if (shrink != 0.0f) {
			Vec3 direction = centroid - v->position; // Direction from vertex to centroid
			newPosition += direction * shrink; // Move the vertex towards or away from the centroid
		}

		// Apply the translation
		newPosition += move;

		// Update the vertex position
		v->position = newPosition;
		face_hg = face_hg->next;
	} while(face_hg != face->halfedge);
}

