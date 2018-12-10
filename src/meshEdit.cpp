#include <float.h>
#include <assert.h>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"

namespace CMU462 {

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should split the given edge and return an iterator to the
  // newly inserted vertex. The halfedge of this vertex should point along
  // the edge that was split, rather than the new edges.

	HalfedgeIter h = e0->halfedge();

	HalfedgeIter h1, h2, h3, h4, h5, h6, h7, h8, h_twin, old_h1, old_h2, old_h3, old_h4;
	FaceIter f1, f2, f3, f4, old_f1, old_f2;
	VertexIter v1, v2, v3;
	EdgeIter e1, e2, e3, e4;

	h_twin = h->twin();
	old_f1 = h->face();
	old_f2 = h_twin->face();
	v2 = h->vertex();
	v3 = h_twin->vertex();
	if (old_f1->degree() > 3 || old_f2->degree() > 3) {
		showError("Not both of faces are triangle!");
		return v2;
	}

	if (e0->isBoundary()) {
		showError("Edge is boundary!");
		return v2;
	}

	old_h1 = h->next();
	old_h2 = old_h1->next();
	old_h3 = h_twin->next();
	old_h4 = old_h3->next();


	h1 = h;
	h2 = h_twin;
	h3 = newHalfedge();
	h4 = newHalfedge();
	h5 = newHalfedge();
	h6 = newHalfedge();
	h7 = newHalfedge();
	h8 = newHalfedge();
	f1 = old_f1;
	f2 = old_f2;
	f3 = newFace();
	f4 = newFace();
	v1 = newVertex();
	e1 = e0;
	e2 = newEdge();
	e3 = newEdge();
	e4 = newEdge();

	v1->halfedge() = h1;
	v1->position = (v2->position + v3->position) / 2;
	e1->halfedge() = h1;
	e2->halfedge() = h3;
	e3->halfedge() = h5;
	e4->halfedge() = h7;
	f1->halfedge() = h1;
	f2->halfedge() = h2;
	f3->halfedge() = h3;
	f4->halfedge() = h4;
	v2->halfedge() = h3;
	v3->halfedge() = h2;

	h1->setNeighbors(old_h1, h2, v1, e1, f1);
	h2->setNeighbors(h7, h1, v3, e1, f2);
	h3->setNeighbors(h6, h4, v2, e2, f3);
	h4->setNeighbors(old_h3, h3, v1, e2, f4);
	h5->setNeighbors(h1, h6, old_h2->vertex(), e3, f1);
	h6->setNeighbors(old_h2, h5, v1, e3, f3);
	h7->setNeighbors(old_h4, h8, v1, e4, f2);
	h8->setNeighbors(h4, h7, old_h4->vertex(), e4, f4);

	old_h1->next() = h5;
	old_h1->face() = f1;
	old_h2->next() = h3;
	old_h2->face() = f3;
	old_h3->next() = h8;
	old_h3->face() = f4;
	old_h4->next() = h2;
	old_h4->face() = f2;

	return v1;
}

VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should collapse the given edge and return an iterator to
  // the new vertex created by the collapse.

	HalfedgeIter h = e->halfedge();
	HalfedgeIter h1, h2, h3, h4, h_twin, h_iter;
	VertexIter v1, v2, v;
	FaceIter f1, f2;

	h_twin = h->twin();
	f1 = h->face();
	f2 = h_twin->face();
	h1 = h->next();
	h3 = h_twin->next();
	v1 = h_twin->vertex();
	v2 = h->vertex();

	if (e->isBoundary()) {
		showError("Edge is boundary!");
		return v2;
	}

	do {
		h2 = h;
		h = h->next();
	} while (h != e->halfedge());
	do {
		h4 = h_twin;
		h_twin = h_twin->next();
	} while (h_twin != h->twin());

	// reassign vertex
	v = newVertex();
	v->halfedge() = h2->twin();
	v->position = (v1->position + v2->position) / 2;
	h_iter = h;
	do {
		h_iter->vertex() = v;
		h_iter = h_iter->twin()->next();
	} while (h_iter != h);
	h_iter = h_twin;
	do {
		h_iter->vertex() = v;
		h_iter = h_iter->twin()->next();
	} while (h_iter != h_twin);

	// means the original polygon is triangle, need to be degraded
	if (h1->next() == h2) {
		HalfedgeIter h1_twin, h2_twin;
		EdgeIter e1, e2;
		e1 = h1->edge();
		e2 = h2->edge();
		h1_twin = h1->twin();
		h2_twin = h2->twin();
		h1_twin->twin() = h2_twin;
		h2_twin->twin() = h1_twin;
		h2_twin->edge() = e1;
		e1->halfedge() = h1_twin;
		e2->halfedge() = h2_twin;
		h2->vertex()->halfedge() = h1_twin;

		deleteHalfedge(h);
		deleteHalfedge(h1);
		deleteHalfedge(h2);
		deleteEdge(e2);
		deleteFace(f1);
	}
	else {
		h2->next() = h1;
		f1->halfedge() = h1;
		deleteHalfedge(h);
	}
	// means the original polygon is triangle, need to be degraded
	if (h3->next() == h4) {
		HalfedgeIter h3_twin, h4_twin;
		EdgeIter e3, e4;
		e3 = h3->edge();
		e4 = h4->edge();
		h3_twin = h3->twin();
		h4_twin = h4->twin();
		h3_twin->twin() = h4_twin;
		h4_twin->twin() = h3_twin;
		h4_twin->edge() = e3;
		e3->halfedge() = h3_twin;
		e4->halfedge() = h4_twin;
		h4->vertex()->halfedge() = h3_twin;

		deleteHalfedge(h_twin);
		deleteHalfedge(h3);
		deleteHalfedge(h4);
		deleteEdge(e4);
		deleteFace(f2);
	}
	else {
		h4->next() = h3;
		f2->halfedge() = h3;
		deleteHalfedge(h_twin);
	}

	deleteEdge(e);
	deleteVertex(v1);
	deleteVertex(v2);
	return v;
}

VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
  // TODO: (meshEdit)
  // This method should collapse the given face and return an iterator to
  // the new vertex created by the collapse.
	vector<EdgeIter> oldEdges;
	vector<HalfedgeIter> oldHalfedges;
	vector<FaceIter> oldFaces;
	vector<VertexIter> oldVertices;
	HalfedgeIter h = f->halfedge();
	HalfedgeIter h0, h1, h2, h_iter;
	EdgeIter e;
	VertexIter v, v1;

	bool hasHalfEdge = false;
	do {
		h0 = h->twin();
		if (h0->face()->degree() > 3) {
			hasHalfEdge = true;
			break;
		}
		h = h->next();
	} while (h != f->halfedge());
	if (!hasHalfEdge) {
		showError("Don't allow degenerate case!");
		return verticesBegin();
	}

	oldFaces.push_back(f);
	v = newVertex();
	v->position = f->centroid();

	// reassign vertex
	do {
		h_iter = h;
		oldVertices.push_back(h->vertex());
		do {
			h_iter->vertex() = v;
			h_iter = h_iter->twin()->next();
		} while (h_iter != h);
		h = h->next();
	} while (h != f->halfedge());

	// deal with each face
	do {
		e = h->edge();
		v1 = h->vertex();
		h0 = h->twin();
		h1 = h0->next();
		do {
			h2 = h0;
			h0 = h0->next();
		} while (h0 != h->twin());


		FaceIter f1 = h0->face();
		// triangle face, degenerate case
		if (h1->next() == h2) {
			HalfedgeIter h1_twin, h2_twin;
			EdgeIter e1, e2;
			e1 = h1->edge();
			e2 = h2->edge();
			h1_twin = h1->twin();
			h2_twin = h2->twin();
			h1_twin->twin() = h2_twin;
			h2_twin->twin() = h1_twin;
			h2_twin->edge() = e1;
			e1->halfedge() = h1_twin;
			e2->halfedge() = h2_twin;
			h2->vertex()->halfedge() = h1_twin;

			oldHalfedges.push_back(h1);
			oldHalfedges.push_back(h2);
			oldEdges.push_back(e2);
			oldFaces.push_back(f1);
		}
		// general case
		else {
			h2->next() = h1;
			f1->halfedge() = h1;
			v->halfedge() = h1;
		}
		oldEdges.push_back(e);
		oldHalfedges.push_back(h);
		oldHalfedges.push_back(h0);
		h = h->next();
	} while (h != f->halfedge());

	for (auto i = oldEdges.begin(); i != oldEdges.end(); i++) {
		deleteEdge(*i);
	}
	for (auto i = oldVertices.begin(); i != oldVertices.end(); i++) {
		deleteVertex(*i);
	}
	for (auto i = oldFaces.begin(); i != oldFaces.end(); i++) {
		deleteFace(*i);
	}
	for (auto i = oldHalfedges.begin(); i != oldHalfedges.end(); i++) {
		deleteHalfedge(*i);
	}
	return v;
}

FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
  // TODO: (meshEdit)
  // This method should replace the given vertex and all its neighboring
  // edges and faces with a single face, returning the new face.

  return FaceIter();
}

FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
  // TODO: (meshEdit)
  // This method should erase the given edge and return an iterator to the
  // merged face.

  showError("eraseVertex() not implemented.");
  return FaceIter();
}

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  // TODO: (meshEdit)
  // This method should flip the given edge and return an iterator to the
  // flipped edge.

	if (e0->isBoundary()) {
		showError("Cannot flip boundary edge!");
		return e0;
	}


	HalfedgeIter h = e0->halfedge();
	HalfedgeIter h1, h2, h3, h4, h5, h6, h_twin;
	VertexIter v1, v2, v3, v4;
	FaceIter f1, f2;

	h1 = h->next();
	h_twin = h->twin();
	h3 = h_twin->next();
	h5 = h1->next();
	h6 = h3->next();
	v1 = h1->twin()->vertex();
	v2 = h3->twin()->vertex();
	v3 = h_twin->vertex();
	v4 = h->vertex();
	f1 = h->face();
	f2 = h_twin->face();

	do {
		h2 = h;
		h = h->next();
	} while (h != e0->halfedge());
	do {
		h4 = h_twin;
		h_twin = h_twin->next();
	} while (h_twin != h->twin());

	f1->halfedge() = h;
	f2->halfedge() = h_twin;
	v3->halfedge() = h1;
	v4->halfedge() = h3;

	h->setNeighbors(h5, h_twin, v2, e0, f1);
	h_twin->setNeighbors(h6, h, v1, e0, f2);
	h1->setNeighbors(h_twin, h1->twin(), h1->vertex(), h1->edge(), f2);
	h2->setNeighbors(h3, h2->twin(), h2->vertex(), h2->edge(), f1);
	h3->setNeighbors(h, h3->twin(), h3->vertex(), h3->edge(), f1);
	h4->setNeighbors(h1, h4->twin(), h4->vertex(), h4->edge(), f2);

	return e0;
}

void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
  // Unlike the local mesh operations (like bevel or edge flip), we will perform
  // subdivision by splitting *all* faces into quads "simultaneously."  Rather
  // than operating directly on the halfedge data structure (which as you've
  // seen
  // is quite difficult to maintain!) we are going to do something a bit nicer:
  //
  //    1. Create a raw list of vertex positions and faces (rather than a full-
  //       blown halfedge mesh).
  //
  //    2. Build a new halfedge mesh from these lists, replacing the old one.
  //
  // Sometimes rebuilding a data structure from scratch is simpler (and even
  // more
  // efficient) than incrementally modifying the existing one.  These steps are
  // detailed below.

  // TODO Step I: Compute the vertex positions for the subdivided mesh.  Here
  // we're
  // going to do something a little bit strange: since we will have one vertex
  // in
  // the subdivided mesh for each vertex, edge, and face in the original mesh,
  // we
  // can nicely store the new vertex *positions* as attributes on vertices,
  // edges,
  // and faces of the original mesh.  These positions can then be conveniently
  // copied into the new, subdivided mesh.
  // [See subroutines for actual "TODO"s]
  if (useCatmullClark) {
    computeCatmullClarkPositions();
  } else {
    computeLinearSubdivisionPositions();
  }

  // TODO Step II: Assign a unique index (starting at 0) to each vertex, edge,
  // and
  // face in the original mesh.  These indices will be the indices of the
  // vertices
  // in the new (subdivided mesh).  They do not have to be assigned in any
  // particular
  // order, so long as no index is shared by more than one mesh element, and the
  // total number of indices is equal to V+E+F, i.e., the total number of
  // vertices
  // plus edges plus faces in the original mesh.  Basically we just need a
  // one-to-one
  // mapping between original mesh elements and subdivided mesh vertices.
  // [See subroutine for actual "TODO"s]
  assignSubdivisionIndices();

  // TODO Step III: Build a list of quads in the new (subdivided) mesh, as
  // tuples of
  // the element indices defined above.  In other words, each new quad should be
  // of
  // the form (i,j,k,l), where i,j,k and l are four of the indices stored on our
  // original mesh elements.  Note that it is essential to get the orientation
  // right
  // here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces should
  // circulate in the same direction as old faces (think about the right-hand
  // rule).
  // [See subroutines for actual "TODO"s]
  vector<vector<Index> > subDFaces;
  vector<Vector3D> subDVertices;
  buildSubdivisionFaceList(subDFaces);
  buildSubdivisionVertexList(subDVertices);

  // TODO Step IV: Pass the list of vertices and quads to a routine that clears
  // the
  // internal data for this halfedge mesh, and builds new halfedge data from
  // scratch,
  // using the two lists.
  rebuild(subDFaces, subDVertices);
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
void HalfedgeMesh::computeLinearSubdivisionPositions() {
  // TODO For each vertex, assign Vertex::newPosition to
  // its original position, Vertex::position.

  // TODO For each edge, assign the midpoint of the two original
  // positions to Edge::newPosition.

  // TODO For each face, assign the centroid (i.e., arithmetic mean)
  // of the original vertex positions to Face::newPosition.  Note
  // that in general, NOT all faces will be triangles!
	for (auto v = vertices.begin(); v != vertices.end(); v++)
	{
		v->newPosition = v->position;
	}
	for (auto e = edges.begin(); e != edges.end(); e++)
	{
		e->newPosition = e->centroid();
	}
	for (auto f = faces.begin(); f != faces.end(); f++)
	{
		f->newPosition = f->centroid();
	}
  //showError("computeLinearSubdivisionPositions() not implemented.");
}

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
void HalfedgeMesh::computeCatmullClarkPositions() {
  // TODO The implementation for this routine should be
  // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
  // except that the calculation of the positions themsevles is
  // slightly more involved, using the Catmull-Clark subdivision
  // rules. (These rules are outlined in the Developer Manual.)

  // TODO face
	for (auto f = faces.begin(); f != faces.end(); f++)
	{
		f->newPosition = f->centroid();
	}

  // TODO edges
	for (auto e = edges.begin(); e != edges.end(); e++)
	{
		auto h = e->halfedge();
		e->newPosition = (e->centroid() * 2 + h->face()->newPosition + h->twin()->face()->newPosition) / 4;
	}
  // TODO vertices
	for (auto v = vertices.begin(); v != vertices.end(); v++)
	{
		auto h = v->halfedge();
		Vector3D Q(0, 0, 0);
		Vector3D R(0, 0, 0);
		Size n = 0;
		do {
			Q += h->face()->newPosition;
			R += h->edge()->newPosition;
			n++;
			h = h->twin()->next();
		} while (h != v->halfedge());
		Q /= n;
		R /= n;
		v->newPosition = (Q + 2 * R + (n - 3)*(v->position)) / n;
	}
  //showError("computeCatmullClarkPositions() not implemented.");
}

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
void HalfedgeMesh::assignSubdivisionIndices() {
  // TODO Start a counter at zero; if you like, you can use the
  // "Index" type (defined in halfedgeMesh.h)

  // TODO Iterate over vertices, assigning values to Vertex::index

  // TODO Iterate over edges, assigning values to Edge::index

  // TODO Iterate over faces, assigning values to Face::index
	Index cnt = 0;
	for (auto v = verticesBegin(); v != verticesEnd(); v++) {
		v->index = cnt;
		cnt++;
	}
	for (auto e = edgesBegin(); e != edgesEnd(); e++) {
		e->index = cnt;
		cnt++;
	}
	for (auto f = facesBegin(); f != facesEnd(); f++) {
		f->index = cnt;
		cnt++;
	}
  //showError("assignSubdivisionIndices() not implemented.");
}

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D>& subDVertices) {
  // TODO Resize the vertex list so that it can hold all the vertices.

  // TODO Iterate over vertices, assigning Vertex::newPosition to the
  // appropriate location in the new vertex list.

  // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
  // location in the new vertex list.

  // TODO Iterate over faces, assigning Face::newPosition to the appropriate
  // location in the new vertex list.
	subDVertices.resize(vertices.size() + edges.size() + faces.size());
	for (auto v = vertices.begin(); v != vertices.end(); v++)
	{
		subDVertices[v->index] = v->newPosition;
	}
	for (auto e = edgesBegin(); e != edgesEnd(); e++) {
		subDVertices[e->index] = e->newPosition;
	}
	for (auto f = facesBegin(); f != facesEnd(); f++) {
		subDVertices[f->index] = f->newPosition;
	}

  //showError("buildSubdivisionVertexList() not implemented.");
}

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index> >& subDFaces) {
  // TODO This routine is perhaps the most tricky step in the construction of
  // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
  // vertex positions).  Basically what you want to do is iterate over faces,
  // then for each for each face, append N quads to the list (where N is the
  // degree of the face).  For this routine, it may be more convenient to simply
  // append quads to the end of the list (rather than allocating it ahead of
  // time), though YMMV.  You can of course iterate around a face by starting
  // with its first halfedge and following the "next" pointer until you get
  // back to the beginning.  The tricky part is making sure you grab the right
  // indices in the right order---remember that there are indices on vertices,
  // edges, AND faces of the original mesh.  All of these should get used.  Also
  // remember that you must have FOUR indices per face, since you are making a
  // QUAD mesh!

  // TODO iterate over faces
  // TODO loop around face
  // TODO build lists of four indices for each sub-quad
  // TODO append each list of four indices to face list
	for (auto f = faces.begin(); f != faces.end(); f++)
	{
		auto h = f->halfedge();
		do
		{
			vector<Index> quad(4);
			quad[0] = h->edge()->index;
			quad[1] = h->next()->vertex()->index;
			quad[2] = h->next()->edge()->index;
			quad[3] = f->index;

			subDFaces.push_back(quad);
			h = h->next();
		} while (h!=f->halfedge());
	}
  //showError("buildSubdivisionFaceList() not implemented.");
}

FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
  // TODO This method should replace the vertex v with a face, corresponding to
  // a bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
  // implement!)

	HalfedgeIter h, h1, h2, h3, h4, pre_h3;
	EdgeIter e1;
	VertexIter v1;
	FaceIter newf, f1;

	newf = newFace();
	h = v->halfedge();
	do
	{
		h1 = h->twin();
		h4 = h1->next();
		f1 = h1->face();
		e1 = newEdge();
		v1 = newVertex();
		h2 = newHalfedge();
		h3 = newHalfedge();

		e1->halfedge() = h2;
		v1->halfedge() = h2;
		h1->next() = h2;
		h2->setNeighbors(h4, h3, v1, e1, f1);
		h3->setNeighbors(h3->next(), h2, h3->vertex(), e1, newf);
		h->vertex() = v1;
		if (h != v->halfedge()) {
			h3->next() = pre_h3;
			pre_h3->vertex() = v1;
		}
		pre_h3 = h3;
		h = h4;
	} while (h !=v->halfedge());

	h3 = h->twin()->next()->twin();
	h3->next() = pre_h3;
	pre_h3->vertex() = h->vertex();
	newf->halfedge() = h3;
	return newf;
}

FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
  // TODO This method should replace the edge e with a face, corresponding to a
  // bevel operation. It should return the new face.  NOTE: This method is
  // responsible for updating the *connectivity* of the mesh only---it does not
  // need to update the vertex positions.  These positions will be updated in
  // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
  // implement!)

  showError("bevelEdge() not implemented.");
  return facesBegin();
}

FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
  // TODO This method should replace the face f with an additional, inset face
  // (and ring of faces around it), corresponding to a bevel operation. It
  // should return the new face.  NOTE: This method is responsible for updating
  // the *connectivity* of the mesh only---it does not need to update the vertex
  // positions.  These positions will be updated in
  // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
  // implement!)

	if (f->isBoundary()) {
		showError("Face is boundary!");
		return f;
	}

	HalfedgeIter h = f->halfedge();

	HalfedgeIter h1, h2, h3, h4, pre_h, pre_h1, pre_h2;
	FaceIter newf, ringf, pre_ringf;
	VertexIter v, v1;
	EdgeIter e1, e2;

	newf = f;

	do {
		v = h->vertex();

		ringf = newFace();
		e1 = newEdge();
		e2 = newEdge();
		v1 = newVertex();
		h1 = newHalfedge();
		h2 = newHalfedge();
		h3 = newHalfedge();
		h4 = newHalfedge();

		// update new face, edges and vertex
		ringf->halfedge() = h;
		e1->halfedge() = h1;
		e2->halfedge() = h3;
		v1->halfedge() = h2;

		// update h
		h->face() = ringf;

		// update h1,h2,h3,h4
		h1->setNeighbors(h3, h2, h1->vertex(), e1, ringf);
		h2->setNeighbors(h2->next(), h1, v1, e1, newf);
		h3->setNeighbors(h, h4, v1, e2, ringf);
		h4->setNeighbors(h4->next(), h3, v, e2, h4->face());

		// update pre_h, pre_h1, pre_h2 and h4
		if (h != f->halfedge()) {
			pre_h->next() = h4;
			pre_h1->vertex() = v1;
			pre_h2->next() = h2;
			h4->next() = pre_h1;
			h4->face() = pre_ringf;
		}
		pre_h = h;
		pre_h1 = h1;
		pre_h2 = h2;
		pre_ringf = ringf;

		h = h->next();
	} while (h != f->halfedge());

	HalfedgeIter cur_h4 = h->next()->next()->next()->twin();
	HalfedgeIter cur_h2 = h->next()->next()->twin();
	pre_h->next() = cur_h4;
	pre_h1->vertex() = cur_h2->vertex();
	pre_h2->next() = cur_h2;
	cur_h4->next() = pre_h1;
	cur_h4->face() = pre_ringf;

	newf->halfedge() = cur_h2;

	return newf;
}


void HalfedgeMesh::bevelFaceComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double normalShift,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled face.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the originalVertexPositions array) to compute an offset vertex
  // position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); hs++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //
	int n = newHalfedges.size();
	if (n == 0) {
		return;
	}
	Vector3D barycenter = originalVertexPositions[0];
	for (int i = 1; i < n; i++)
	{
		barycenter += originalVertexPositions[i];
	}
	barycenter = barycenter / n;

	// compute face normal
	Vector3D normal(0., 0., 0.);
	for (int i = 0; i < n; i++) {
		Vector3D pi = originalVertexPositions[i];
		Vector3D pj = originalVertexPositions[(i + 1) % n];

		normal += cross(pi, pj);
	}
	normal = normal.unit();

	for (int i = 0; i < n; i++)
	{
		//cout << newHalfedges[i]->twin()->vertex()->position << endl;
		//cout << originalVertexPositions[i] << endl;
		Vector3D dis = originalVertexPositions[i] - barycenter;
		newHalfedges[i]->vertex()->position = barycenter + max(0., (0.5 - tangentialInset)) * dis + normalShift * normal;
	}
	return;

}

void HalfedgeMesh::bevelVertexComputeNewPositions(
    Vector3D originalVertexPosition, vector<HalfedgeIter>& newHalfedges,
    double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled vertex.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., hs.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
	int n = newHalfedges.size();
	for (int i = 0; i < n; i++)
	{
		Vector3D dis = newHalfedges[i]->twin()->vertex()->position - originalVertexPosition;
		newHalfedges[i]->vertex()->position = originalVertexPosition + min(1., abs(tangentialInset)) * dis;
	}
}

void HalfedgeMesh::bevelEdgeComputeNewPositions(
    vector<Vector3D>& originalVertexPositions,
    vector<HalfedgeIter>& newHalfedges, double tangentialInset) {
  // TODO Compute new vertex positions for the vertices of the beveled edge.
  //
  // These vertices can be accessed via newHalfedges[i]->vertex()->position for
  // i = 1, ..., newHalfedges.size()-1.
  //
  // The basic strategy here is to loop over the list of outgoing halfedges,
  // and use the preceding and next vertex position from the original mesh
  // (in the orig array) to compute an offset vertex position.
  //
  // Note that there is a 1-to-1 correspondence between halfedges in
  // newHalfedges and vertex positions
  // in orig.  So, you can write loops of the form
  //
  // for( int i = 0; i < newHalfedges.size(); i++ )
  // {
  //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
  //    position correponding to vertex i
  // }
  //

}

void HalfedgeMesh::splitPolygons(vector<FaceIter>& fcs) {
  for (auto f : fcs) splitPolygon(f);
}

void HalfedgeMesh::splitPolygon(FaceIter f) {
  // TODO: (meshedit) 
  // Triangulate a polygonal face
	if (f->degree() <= 3) {
		return;
	}

	HalfedgeIter h = f->halfedge();

	HalfedgeIter h_next, h1, h2, pre_h2;
	EdgeIter e1;
	FaceIter f1;
	VertexIter v0, v1;

	v0 = h->vertex();

	pre_h2 = h;
	h = h->next();
	h_next = h->next();

	while (h_next->twin()->vertex() != v0) {
		e1 = newEdge();
		f1 = newFace();
		h1 = newHalfedge();
		h2 = newHalfedge();

		v1 = h_next->vertex();
		e1->halfedge() = h1;
		f1->halfedge() = h1;
		h->next() = h1;
		h->face() = f1;
		pre_h2->face() = f1;
		h1->setNeighbors(pre_h2, h2, v1, e1, f1);
		h2->setNeighbors(h_next, h1, v0, e1, h2->face());

		pre_h2 = h2;
		h = h_next;
		h_next = h_next->next();
	}
	f->halfedge() = h;
	pre_h2->face() = f;
	h->face() = f;
	h_next->face() = f;
	h_next->next() = pre_h2;
}

EdgeRecord::EdgeRecord(EdgeIter& _edge) : edge(_edge) {
  // TODO: (meshEdit)
  // Compute the combined quadric from the edge endpoints.
  // -> Build the 3x3 linear system whose solution minimizes the quadric error
  //    associated with these two endpoints.
  // -> Use this system to solve for the optimal position, and store it in
  //    EdgeRecord::optimalPoint.
  // -> Also store the cost associated with collapsing this edg in
  //    EdgeRecord::Cost.
}

void MeshResampler::upsample(HalfedgeMesh& mesh)
// This routine should increase the number of triangles in the mesh using Loop
// subdivision.
{
  // TODO: (meshEdit)
  // Compute new positions for all the vertices in the input mesh, using
  // the Loop subdivision rule, and store them in Vertex::newPosition.
  // -> At this point, we also want to mark each vertex as being a vertex of the
  //    original mesh.
  // -> Next, compute the updated vertex positions associated with edges, and
  //    store it in Edge::newPosition.
  // -> Next, we're going to split every edge in the mesh, in any order.  For
  //    future reference, we're also going to store some information about which
  //    subdivided edges come from splitting an edge in the original mesh, and
  //    which edges are new, by setting the flat Edge::isNew. Note that in this
  //    loop, we only want to iterate over edges of the original mesh.
  //    Otherwise, we'll end up splitting edges that we just split (and the
  //    loop will never end!)
  // -> Now flip any new edge that connects an old and new vertex.
  // -> Finally, copy the new vertex positions into final Vertex::position.

  // Each vertex and edge of the original surface can be associated with a
  // vertex in the new (subdivided) surface.
  // Therefore, our strategy for computing the subdivided vertex locations is to
  // *first* compute the new positions
  // using the connectity of the original (coarse) mesh; navigating this mesh
  // will be much easier than navigating
  // the new subdivided (fine) mesh, which has more elements to traverse.  We
  // will then assign vertex positions in
  // the new mesh based on the values we computed for the original mesh.

  // Compute updated positions for all the vertices in the original mesh, using
  // the Loop subdivision rule.
	for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++)
	{
		v->isNew = false;
		Vector3D P(0, 0, 0);
		size_t n = 0;
		auto h = v->halfedge();
		do
		{
			P += h->twin()->vertex()->position;
			n++;
			h = h->twin()->next();
		} while (h != v->halfedge());

		v->newPosition = (3 * P) / (8 * n) + (5 * v->position) / 8;
	}

  // Next, compute the updated vertex positions associated with edges.
	for (auto e = mesh.edgesBegin(); e !=mesh.edgesEnd(); e++)
	{
		e->isNew = false;
		auto h = e->halfedge();
		e->newPosition = (h->vertex()->position + h->twin()->vertex()->position) * 3 / 8 + (h->next()->twin()->vertex()->position + h->twin()->next()->twin()->vertex()->position) / 8;
	}

  // Next, we're going to split every edge in the mesh, in any order.  For
  // future
  // reference, we're also going to store some information about which
  // subdivided
  // edges come from splitting an edge in the original mesh, and which edges are
  // new.
  // In this loop, we only want to iterate over edges of the original
  // mesh---otherwise,
  // we'll end up splitting edges that we just split (and the loop will never
  // end!)
	int n = mesh.nEdges();
	auto e = mesh.edgesBegin();
	for (int i = 0; i < n; i++) {
		auto nextEdge = e;
		nextEdge++;
		auto v = mesh.splitEdge(e);
		v->isNew = true;
		auto h = v->halfedge();
		h->edge()->isNew = false;
		h = h->twin()->next();
		h->edge()->isNew = true;
		h = h->twin()->next();
		h->edge()->isNew = false;
		h = h->twin()->next();
		h->edge()->isNew = true;
		v->newPosition = e->newPosition;
		e = nextEdge;
	}

  // Finally, flip any new edge that connects an old and new vertex.
	for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++)
	{
		if (e->isNew && (e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew))
		{
			mesh.flipEdge(e);
		}
	}
  // Copy the updated vertex positions to the subdivided mesh.
	for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
		v->position = v->newPosition;
	}
  //showError("upsample() not implemented.");
}

void MeshResampler::downsample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute initial quadrics for each face by simply writing the plane equation
  // for the face in homogeneous coordinates. These quadrics should be stored
  // in Face::quadric
  // -> Compute an initial quadric for each vertex as the sum of the quadrics
  //    associated with the incident faces, storing it in Vertex::quadric
  // -> Build a priority queue of edges according to their quadric error cost,
  //    i.e., by building an EdgeRecord for each edge and sticking it in the
  //    queue.
  // -> Until we reach the target edge budget, collapse the best edge. Remember
  //    to remove from the queue any edge that touches the collapsing edge
  //    BEFORE it gets collapsed, and add back into the queue any edge touching
  //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
  //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
  //    top of the queue.
  showError("downsample() not implemented.");
}

void MeshResampler::resample(HalfedgeMesh& mesh) {
  // TODO: (meshEdit)
  // Compute the mean edge length.
  // Repeat the four main steps for 5 or 6 iterations
  // -> Split edges much longer than the target length (being careful about
  //    how the loop is written!)
  // -> Collapse edges much shorter than the target length.  Here we need to
  //    be EXTRA careful about advancing the loop, because many edges may have
  //    been destroyed by a collapse (which ones?)
  // -> Now flip each edge if it improves vertex degree
  // -> Finally, apply some tangential smoothing to the vertex positions
  showError("resample() not implemented.");
}

}  // namespace CMU462
