/*
* SceneReconstruct.cpp
*
* Copyright (c) 2014-2015 SEACAVE
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Affero General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
* Additional Terms:
*
*      You are required to preserve legal notices and author attributions in
*      that material or in the Appropriate Legal Notices displayed by works
*      containing it.
*/

#include "Common.h"
#include "Scene.h"
// Delaunay: mesh reconstruction
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_cell_base_with_info_3.h>
#include <CGAL/Spatial_sort_traits_adapter_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Polyhedron_3.h>

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define DELAUNAY_USE_OPENMP
#endif

// uncomment to enable reconstruction algorithm of weakly supported surfaces
#define DELAUNAY_WEAKSURF

// uncomment to use IBFS algorithm for max-flow
// (faster, but not clear license policy)
#define DELAUNAY_MAXFLOW_IBFS


// S T R U C T S ///////////////////////////////////////////////////

#ifdef DELAUNAY_MAXFLOW_IBFS
#include "../Math/IBFS/IBFS.h"
template <typename NType, typename VType>
class MaxFlow
{
public:
	// Type-Definitions
	typedef NType node_type;
	typedef VType value_type;
	typedef IBFS::IBFSGraph graph_type;

public:
	MaxFlow(size_t numNodes) {
		graph.initSize((int)numNodes, (int)numNodes*2);
	}

	inline void AddNode(node_type n, value_type source, value_type sink) {
		ASSERT(ISFINITE(source) && source >= 0 && ISFINITE(sink) && sink >= 0);
		graph.addNode((int)n, source, sink);
	}

	inline void AddEdge(node_type n1, node_type n2, value_type capacity, value_type reverseCapacity) {
		ASSERT(ISFINITE(capacity) && capacity >= 0 && ISFINITE(reverseCapacity) && reverseCapacity >= 0);
		graph.addEdge((int)n1, (int)n2, capacity, reverseCapacity);
	}

	value_type ComputeMaxFlow() {
		graph.initGraph();
		return graph.computeMaxFlow();
	}

	inline bool IsNodeOnSrcSide(node_type n) const {
		return graph.isNodeOnSrcSide((int)n);
	}

protected:
	graph_type graph;
};
#else
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
template <typename NType, typename VType>
class MaxFlow
{
public:
	// Type-Definitions
	typedef NType node_type;
	typedef VType value_type;
	typedef boost::vecS out_edge_list_t;
	typedef boost::vecS vertex_list_t;
	typedef boost::adjacency_list_traits<out_edge_list_t, vertex_list_t, boost::directedS> graph_traits;
	typedef typename graph_traits::edge_descriptor edge_descriptor;
	typedef typename graph_traits::vertex_descriptor vertex_descriptor;
	typedef typename graph_traits::vertices_size_type vertex_size_type;
	struct Edge {
		value_type capacity;
		value_type residual;
		edge_descriptor reverse;
	};
	typedef boost::adjacency_list<out_edge_list_t, vertex_list_t, boost::directedS, size_t, Edge> graph_type;
	typedef typename boost::graph_traits<graph_type>::edge_iterator edge_iterator;
	typedef typename boost::graph_traits<graph_type>::out_edge_iterator out_edge_iterator;

public:
	MaxFlow(size_t numNodes) : graph(numNodes+2), S(node_type(numNodes)), T(node_type(numNodes+1)) {}

	void AddNode(node_type n, value_type source, value_type sink) {
		ASSERT(ISFINITE(source) && source >= 0 && ISFINITE(sink) && sink >= 0);
		if (source > 0) {
			edge_descriptor e(boost::add_edge(S, n, graph).first);
			edge_descriptor er(boost::add_edge(n, S, graph).first);
			graph[e].capacity = source;
			graph[e].reverse = er;
			graph[er].reverse = e;
		}
		if (sink > 0) {
			edge_descriptor e(boost::add_edge(n, T, graph).first);
			edge_descriptor er(boost::add_edge(T, n, graph).first);
			graph[e].capacity = sink;
			graph[e].reverse = er;
			graph[er].reverse = e;
		}
	}

	void AddEdge(node_type n1, node_type n2, value_type capacity, value_type reverseCapacity) {
		ASSERT(ISFINITE(capacity) && capacity >= 0 && ISFINITE(reverseCapacity) && reverseCapacity >= 0);
		edge_descriptor e(boost::add_edge(n1, n2, graph).first);
		edge_descriptor er(boost::add_edge(n2, n1, graph).first);
		graph[e].capacity = capacity;
		graph[er].capacity = reverseCapacity;
		graph[e].reverse = er;
		graph[er].reverse = e;
	}

	value_type ComputeMaxFlow() {
		vertex_size_type n_verts(boost::num_vertices(graph));
		color.resize(n_verts);
		std::vector<edge_descriptor> pred(n_verts);
		std::vector<vertex_size_type> dist(n_verts);
		return boost::boykov_kolmogorov_max_flow(graph,
			boost::get(&Edge::capacity, graph),
			boost::get(&Edge::residual, graph),
			boost::get(&Edge::reverse, graph),
			&pred[0],
			&color[0],
			&dist[0],
			boost::get(boost::vertex_index, graph),
			S, T
		);
	}

	inline bool IsNodeOnSrcSide(node_type n) const {
		return (color[n] != boost::white_color);
	}

protected:
	graph_type graph;
	std::vector<boost::default_color_type> color;
	const node_type S;
	const node_type T;
};
#endif
/*----------------------------------------------------------------*/


// S T R U C T S ///////////////////////////////////////////////////

// construct the mesh out of the dense point cloud using Delaunay tetrahedralization & graph-cut method
// see "Exploiting Visibility Information in Surface Reconstruction to Preserve Weakly Supported Surfaces", Jancosek and Pajdla, 2015
namespace DELAUNAY {
typedef CGAL::Exact_predicates_inexact_constructions_kernel kernel_t;
typedef kernel_t::Point_3 point_t;
typedef kernel_t::Vector_3 vector_t;
typedef kernel_t::Direction_3 direction_t;
typedef kernel_t::Segment_3 segment_t;
typedef kernel_t::Plane_3 plane_t;
typedef kernel_t::Triangle_3 triangle_t;
typedef kernel_t::Ray_3 ray_t;

typedef uint32_t vert_size_t;
typedef uint32_t cell_size_t;

typedef float edge_cap_t;

#ifdef DELAUNAY_WEAKSURF
struct view_info_t;
#endif
struct vert_info_t {
	typedef edge_cap_t Type;
	struct view_t {
		PointCloud::View idxView; // view index
		Type weight; // point's weight
		inline view_t() {}
		inline view_t(PointCloud::View _idxView, Type _weight) : idxView(_idxView), weight(_weight) {}
		inline bool operator <(const view_t& v) const { return idxView < v.idxView; }
		inline operator PointCloud::View() const { return idxView; }
	};
	typedef SEACAVE::cList<view_t,const view_t&,0,4,uint32_t> view_vec_t;
	view_vec_t views; // faces' weight from the cell outwards
	#ifdef DELAUNAY_WEAKSURF
	view_info_t* viewsInfo; // each view caches the two faces from the point towards the camera and the end (used only by the weakly supported surfaces)
	inline vert_info_t() : viewsInfo(NULL) {}
	~vert_info_t();
	void AllocateInfo();
	#else
	inline vert_info_t() {}
	#endif
	void InsertViews(const PointCloud& pc, PointCloud::Index idxPoint) {
		const PointCloud::ViewArr& _views = pc.pointViews[idxPoint];
		ASSERT(!_views.IsEmpty());
		const PointCloud::WeightArr* pweights(pc.pointWeights.IsEmpty() ? NULL : pc.pointWeights.Begin()+idxPoint);
		ASSERT(pweights == NULL || _views.GetSize() == pweights->GetSize());
		FOREACH(i, _views) {
			const PointCloud::View viewID(_views[i]);
			const PointCloud::Weight weight(pweights ? (*pweights)[i] : PointCloud::Weight(1));
			// insert viewID in increasing order
			const uint32_t idx(views.FindFirstEqlGreater(viewID));
			if (idx < views.GetSize() && views[idx] == viewID) {
				// the new view is already in the array
				ASSERT(views.FindFirst(viewID) == idx);
				// update point's weight
				views[idx].weight += weight;
			} else {
				// the new view is not in the array,
				// insert it
				views.InsertAt(idx, view_t(viewID, weight));
				ASSERT(views.IsSorted());
			}
		}
	}
};

struct cell_info_t {
	typedef edge_cap_t Type;
	Type f[4]; // faces' weight from the cell outwards
	Type s; // cell's weight towards s-source
	Type t; // cell's weight towards t-sink
	inline const Type* ptr() const { return f; }
	inline Type* ptr() { return f; }
};

typedef CGAL::Triangulation_vertex_base_with_info_3<vert_info_t, kernel_t> vertex_base_t;
typedef CGAL::Triangulation_cell_base_with_info_3<cell_size_t, kernel_t> cell_base_t;
typedef CGAL::Triangulation_data_structure_3<vertex_base_t, cell_base_t> triangulation_data_structure_t;
typedef CGAL::Delaunay_triangulation_3<kernel_t, triangulation_data_structure_t, CGAL::Compact_location> delaunay_t;
typedef delaunay_t::Vertex_handle vertex_handle_t;
typedef delaunay_t::Cell_handle cell_handle_t;
typedef delaunay_t::Facet facet_t;
typedef delaunay_t::Edge edge_t;

#ifdef DELAUNAY_WEAKSURF
struct view_info_t {
	cell_handle_t cell2Cam;
	cell_handle_t cell2End;
};
vert_info_t::~vert_info_t() {
	delete[] viewsInfo;
}
void vert_info_t::AllocateInfo() {
	ASSERT(!views.IsEmpty());
	viewsInfo = new view_info_t[views.GetSize()];
	#ifndef _RELEASE
	memset(viewsInfo, 0, sizeof(view_info_t)*views.GetSize());
	#endif
}
#endif

struct camera_cell_t {
	cell_handle_t cell; // cell containing the camera
	std::vector<facet_t> facets; // all facets on the convex-hull in view of the camera (ordered by importance)
};

struct adjacent_vertex_back_inserter_t {
	const delaunay_t& delaunay;
	const point_t& p;
	vertex_handle_t& v;
	inline adjacent_vertex_back_inserter_t(const delaunay_t& _delaunay, const point_t& _p, vertex_handle_t& _v) : delaunay(_delaunay), p(_p), v(_v) {}
	inline adjacent_vertex_back_inserter_t& operator*() { return *this; }
	inline adjacent_vertex_back_inserter_t& operator++(int) { return *this; }
	inline void operator=(const vertex_handle_t& w) {
		ASSERT(!delaunay.is_infinite(v));
		if (!delaunay.is_infinite(w) && delaunay.geom_traits().compare_distance_3_object()(p, w->point(), v->point()) == CGAL::SMALLER)
			v = w;
	}
};

typedef TPoint3<kernel_t::RT> DPoint3;
template <typename TYPE>
inline TPoint3<TYPE> CGAL2MVS(const point_t& p) {
	return TPoint3<TYPE>((TYPE)p.x(), (TYPE)p.y(), (TYPE)p.z());
}
template <typename TYPE>
inline point_t MVS2CGAL(const TPoint3<TYPE>& p) {
	return point_t((kernel_t::RT)p.x, (kernel_t::RT)p.y, (kernel_t::RT)p.z);
}

// Given a facet, compute the plane containing it
inline Plane getFacetPlane(const facet_t& facet)
{
	const point_t& v0(facet.first->vertex((facet.second+1)%4)->point());
	const point_t& v1(facet.first->vertex((facet.second+2)%4)->point());
	const point_t& v2(facet.first->vertex((facet.second+3)%4)->point());
	return Plane(CGAL2MVS<REAL>(v0), CGAL2MVS<REAL>(v1), CGAL2MVS<REAL>(v2));
}


// Check if a point (p) is coplanar with a triangle (a, b, c);
// return orientation type
#ifdef __GNUC__
#pragma GCC push_options
#pragma GCC target ("no-fma")
#endif
static inline int orientation(const point_t& a, const point_t& b, const point_t& c, const point_t& p)
{
	#if 0
	return CGAL::orientation(a, b, c, p);
	#else
	// inexact_orientation
	const double& px = a.x(); const double& py = a.y(); const double& pz = a.z();
	const double pqx(b.x()-px); const double prx(c.x()-px); const double psx(p.x()-px);
	const double pqy(b.y()-py); const double pry(c.y()-py); const double psy(p.y()-py);
	#if 1
	const double det((pqx*pry-prx*pqy)*(p.z()-pz) - (pqx*psy-psx*pqy)*(c.z()-pz) + (prx*psy-psx*pry)*(b.z()-pz));
	const double eps(1e-12);
	#else // very slow due to ABS()
	const double pqz(b.z()-pz); const double prz(c.z()-pz); const double psz(p.z()-pz);
	const double det(CGAL::determinant(
		pqx, pqy, pqz,
		prx, pry, prz,
		psx, psy, psz));
	const double max0(MAXF3(ABS(pqx), ABS(pqy), ABS(pqz)));
	const double max1(MAXF3(ABS(prx), ABS(pry), ABS(prz)));
	const double eps(5.1107127829973299e-15 * MAXF(max0, max1));
	#endif
	if (det >  eps) return CGAL::POSITIVE;
	if (det < -eps) return CGAL::NEGATIVE;
	return CGAL::COPLANAR;
	#endif
}
#ifdef __GNUC__
#pragma GCC pop_options
#endif

// Check if a point (p) is inside a frustum
// given the four corners (a, b, c, d) and the origin (o) of the frustum
inline bool checkPointInside(const point_t& a, const point_t& b, const point_t& c, const point_t& d, const point_t& o, const point_t& p)
{
	return (
		orientation(o, a, b, p) == CGAL::POSITIVE &&
		orientation(o, b, c, p) == CGAL::POSITIVE &&
		orientation(o, c, d, p) == CGAL::POSITIVE &&
		orientation(o, d, a, p) == CGAL::POSITIVE
	);
}

// Given a cell and a camera inside it, if the cell is infinite,
// find all facets on the convex-hull and inside the camera frustum,
// else return all four cell's facets
template <int FacetOrientation>
void fetchCellFacets(const delaunay_t& Tr, const std::vector<facet_t>& hullFacets, const cell_handle_t& cell, const Image& imageData, std::vector<facet_t>& facets)
{
	if (!Tr.is_infinite(cell)) {
		// store all 4 facets of the cell
		for (int i=0; i<4; ++i) {
			const facet_t f(cell, i);
			ASSERT(!Tr.is_infinite(f));
			facets.push_back(f);
		}
		return;
	}
	// find all facets on the convex-hull in camera's view
	// create the 4 frustum planes
	ASSERT(facets.empty());
	typedef TFrustum<REAL,4> Frustum;
	Frustum frustum(imageData.camera.P, imageData.width, imageData.height, 0, 1);
	// loop over all cells
	const point_t ptOrigin(MVS2CGAL(imageData.camera.C));
	for (const facet_t& face: hullFacets) {
		// add face if visible
		const triangle_t verts(Tr.triangle(face));
		if (orientation(verts[0], verts[1], verts[2], ptOrigin) != FacetOrientation)
			continue;
		AABB3 ab(CGAL2MVS<REAL>(verts[0]));
		for (int i=1; i<3; ++i)
			ab.Insert(CGAL2MVS<REAL>(verts[i]));
		if (frustum.Classify(ab) == CULLED)
			continue;
		facets.push_back(face);
	}
}


// information about an intersection between a segment and a facet
struct intersection_t {
	enum Type {FACET, EDGE, VERTEX};
	cell_handle_t ncell; // cell neighbor to the last intersected facet
	vertex_handle_t v1; // vertex for vertex intersection, 1st edge vertex for edge intersection
	vertex_handle_t v2; // 2nd edge vertex for edge intersection
	facet_t facet; // intersected facet
	Type type; // type of intersection (inside facet, on edge, or vertex)
	REAL dist; // distance from starting point (camera) to this facet
	bool bigger; // are we advancing away or towards the starting point?
	const Ray3 ray; // the ray from starting point into the direction of the end point (point -> camera/end-point)
	inline intersection_t() {}
	inline intersection_t(const Point3& pt, const Point3& dir) : dist(-FLT_MAX), bigger(true), ray(pt, dir) {}
};

// Check if a segment (p, q) is coplanar with edges of a triangle (a, b, c):
//  coplanar [in,out] : pointer to the 3 int array of indices of the edges coplanar with pq
// return number of entries in coplanar
inline int checkEdges(const point_t& a, const point_t& b, const point_t& c, const point_t& p, const point_t& q, int coplanar[3])
{
	int nCoplanar(0);
	switch (orientation(p,q,a,b)) {
	case CGAL::POSITIVE: return -1;
	case CGAL::COPLANAR: coplanar[nCoplanar++] = 0;
	}
	switch (orientation(p,q,b,c)) {
	case CGAL::POSITIVE: return -1;
	case CGAL::COPLANAR: coplanar[nCoplanar++] = 1;
	}
	switch (orientation(p,q,c,a)) {
	case CGAL::POSITIVE: return -1;
	case CGAL::COPLANAR: coplanar[nCoplanar++] = 2;
	}
	return nCoplanar;
}

// Check intersection between a facet (f) and a segment (s)
// (derived from CGAL::do_intersect in CGAL/Triangle_3_Segment_3_do_intersect.h)
//  coplanar [out] : pointer to the 3 int array of indices of the edges coplanar with (s)
// return -1 if there is no intersection or
// the number of edges coplanar with the segment (0 = intersection inside the triangle)
int intersect(const triangle_t& t, const segment_t& s, int coplanar[3])
{
	const point_t& a = t.vertex(0);
	const point_t& b = t.vertex(1);
	const point_t& c = t.vertex(2);
	const point_t& p = s.source();
	const point_t& q = s.target();

	switch (orientation(a,b,c,p)) {
	case CGAL::POSITIVE:
		switch (orientation(a,b,c,q)) {
		case CGAL::POSITIVE:
			// the segment lies in the positive open halfspaces defined by the
			// triangle's supporting plane
			return -1;
		case CGAL::COPLANAR:
			// q belongs to the triangle's supporting plane
			// p sees the triangle in counterclockwise order
			return checkEdges(a,b,c,p,q,coplanar);
		case CGAL::NEGATIVE:
			// p sees the triangle in counterclockwise order
			return checkEdges(a,b,c,p,q,coplanar);
		default:
			break;
		}
	case CGAL::NEGATIVE:
		switch (orientation(a,b,c,q)) {
		case CGAL::POSITIVE:
			// q sees the triangle in counterclockwise order
			return checkEdges(a,b,c,q,p,coplanar);
		case CGAL::COPLANAR:
			// q belongs to the triangle's supporting plane
			// p sees the triangle in clockwise order
			return checkEdges(a,b,c,q,p,coplanar);
		case CGAL::NEGATIVE:
			// the segment lies in the negative open halfspaces defined by the
			// triangle's supporting plane
			return -1;
		default:
			break;
		}
	case CGAL::COPLANAR: // p belongs to the triangle's supporting plane
		switch (orientation(a,b,c,q)) {
		case CGAL::POSITIVE:
			// q sees the triangle in counterclockwise order
			return checkEdges(a,b,c,q,p,coplanar);
		case CGAL::COPLANAR:
			// the segment is coplanar with the triangle's supporting plane
			// as we know that it is inside the tetrahedron it intersects the face
			//coplanar[0] = coplanar[1] = coplanar[2] = 3;
			return 3;
		case CGAL::NEGATIVE:
			// q sees the triangle in clockwise order
			return checkEdges(a,b,c,p,q,coplanar);
		default:
			break;
		}
	}
	ASSERT("should not happen" == NULL);
	return -1;
}

// Find which facet is intersected by the segment (seg) and return next facets to check:
//  in_facets [in] : vector of facets to check
//  out_facets [out] : vector of facets to check at next step (can be in_facets)
//  out_inter [out] : kind of intersection
// return false if no intersection found and the end of the segment was not reached
bool intersect(const delaunay_t& Tr, const segment_t& seg, const std::vector<facet_t>& in_facets, std::vector<facet_t>& out_facets, intersection_t& inter)
{
	ASSERT(!in_facets.empty());
	static const int facet_vertex_order[] = {2,1,3,2,2,3,0,2,0,3,1,0,0,1,2,0};
	int coplanar[3];
	const REAL prevDist(inter.dist);
	for (const facet_t& in_facet: in_facets) {
		ASSERT(!Tr.is_infinite(in_facet));
		const int nb_coplanar(intersect(Tr.triangle(in_facet), seg, coplanar));
		if (nb_coplanar >= 0) {
			// skip this cell if the intersection is not in the desired direction
			const REAL interDist(inter.ray.IntersectsDist(getFacetPlane(in_facet)));
			if ((interDist > prevDist) != inter.bigger)
				continue;
			// vertices of facet i: j = 4 * i, vertices = facet_vertex_order[j,j+1,j+2] negative orientation
			inter.facet = in_facet;
			inter.dist = interDist;
			switch (nb_coplanar) {
			case 0: {
				// face intersection
				inter.type = intersection_t::FACET;
				// now find next facets to be checked as
				// the three faces in the neighbor cell different than the origin face
				out_facets.clear();
				const cell_handle_t nc(inter.facet.first->neighbor(inter.facet.second));
				ASSERT(!Tr.is_infinite(nc));
				for (int i=0; i<4; ++i)
					if (nc->neighbor(i) != inter.facet.first)
						out_facets.push_back(facet_t(nc, i));
				return true; }
			case 1: {
				// coplanar with 1 edge = intersect edge
				const int j(4 * inter.facet.second);
				const int i1(j + coplanar[0]);
				inter.type = intersection_t::EDGE;
				inter.v1 = inter.facet.first->vertex(facet_vertex_order[i1+0]);
				inter.v2 = inter.facet.first->vertex(facet_vertex_order[i1+1]);
				// now find next facets to be checked as
				// the two faces in this cell opposing this edge
				out_facets.clear();
				const edge_t out_edge(inter.facet.first, facet_vertex_order[i1+0], facet_vertex_order[i1+1]);
				const typename delaunay_t::Cell_circulator efc(Tr.incident_cells(out_edge));
				typename delaunay_t::Cell_circulator ifc(efc);
				do {
					const cell_handle_t c(ifc);
					if (c == inter.facet.first) continue;
					const facet_t f1(c, c->index(inter.v1));
					if (!Tr.is_infinite(f1))
						out_facets.push_back(f1);
					const facet_t f2(c, c->index(inter.v2));
					if (!Tr.is_infinite(f2))
						out_facets.push_back(f2);
				} while (++ifc != efc);
				return true; }
			case 2: {
				// coplanar with 2 edges = hit a vertex
				// find vertex index
				const int j(4 * inter.facet.second);
				const int i1(j + coplanar[0]);
				const int i2(j + coplanar[1]);
				int i;
				if (facet_vertex_order[i1] == facet_vertex_order[i2] || facet_vertex_order[i1] == facet_vertex_order[i2+1]) {
					i = facet_vertex_order[i1];
				} else
				if (facet_vertex_order[i1+1] == facet_vertex_order[i2] || facet_vertex_order[i1+1] == facet_vertex_order[i2+1]) {
					i = facet_vertex_order[i1+1];
				} else {
					ASSERT("2 edges intersections without common vertex" == NULL);
				}
				inter.type = intersection_t::VERTEX;
				inter.v1 = inter.facet.first->vertex(i);
				ASSERT(!Tr.is_infinite(inter.v1));
				if (inter.v1->point() == seg.target()) {
					// target reached
					out_facets.clear();
					return false;
				}
				// now find next facets to be checked as
				// the faces in the cells around opposing this common vertex
				out_facets.clear();
				struct cell_back_inserter_t {
					const delaunay_t& Tr;
					const vertex_handle_t v;
					const cell_handle_t current_cell;
					std::vector<facet_t>& out_facets;
					inline cell_back_inserter_t(const delaunay_t& _Tr, const intersection_t& inter, std::vector<facet_t>& _out_facets)
						: Tr(_Tr), v(inter.v1), current_cell(inter.facet.first), out_facets(_out_facets) {}
					inline cell_back_inserter_t& operator*() { return *this; }
					inline cell_back_inserter_t& operator++(int) { return *this; }
					inline void operator=(cell_handle_t c) {
						if (c == current_cell)
							return;
						const facet_t f(c, c->index(v));
						if (Tr.is_infinite(f))
							return;
						out_facets.push_back(f);
					}
				};
				Tr.finite_incident_cells(inter.v1, cell_back_inserter_t(Tr, inter, out_facets));
				return true; }
			}
			// coplanar with 3 edges = tangent = impossible?
			break;
		}
	}
	// Bad end: no intersection found and we are not at the end of the segment (very rarely, but it happens)!
	out_facets.clear();
	return false;
}

// same as above, but simplified only to find face intersection (otherwise terminate);
// terminate if cell containing the segment endpoint is found or if an infinite cell is encountered
bool intersectFace(const delaunay_t& Tr, const segment_t& seg, const std::vector<facet_t>& in_facets, std::vector<facet_t>& out_facets, intersection_t& inter)
{
	int coplanar[3];
	for (std::vector<facet_t>::const_iterator it=in_facets.cbegin(); it!=in_facets.cend(); ++it) {
		ASSERT(!Tr.is_infinite(*it));
		if (intersect(Tr.triangle(*it), seg, coplanar) == 0) {
			// face intersection
			inter.facet = *it;
			inter.type = intersection_t::FACET;
			// now find next facets to be checked as
			// the three faces in the neighbor cell different than the origin face
			out_facets.clear();
			inter.ncell = inter.facet.first->neighbor(inter.facet.second);
			if (Tr.is_infinite(inter.ncell))
				return false;
			for (int i=0; i<4; ++i)
				if (inter.ncell->neighbor(i) != inter.facet.first)
					out_facets.push_back(facet_t(inter.ncell, i));
			return true;
		}
	}
	out_facets.clear();
	return false;
}
// same as above, but starts from a known vertex and incident cell
inline bool intersectFace(const delaunay_t& Tr, const segment_t& seg, const vertex_handle_t& v, const cell_handle_t& cell, std::vector<facet_t>& out_facets, intersection_t& inter)
{
	if (cell == cell_handle_t())
		return false;
	if (Tr.is_infinite(cell)) {
		inter.ncell = inter.facet.first = cell;
		return true;
	}
	std::vector<facet_t>& in_facets = out_facets;
	ASSERT(in_facets.empty());
	in_facets.push_back(facet_t(cell, cell->index(v)));
	return intersectFace(Tr, seg, in_facets, out_facets, inter);
}


// Given a cell, compute the free-space support for it
edge_cap_t freeSpaceSupport(const delaunay_t& Tr, const std::vector<cell_info_t>& infoCells, const cell_handle_t& cell)
{
	// sum up all 4 incoming weights
	// (corresponding to the 4 facets of the neighbor cells)
	edge_cap_t wf(0);
	for (int i=0; i<4; ++i) {
		const facet_t& mfacet(Tr.mirror_facet(facet_t(cell, i)));
		wf += infoCells[mfacet.first->info()].f[mfacet.second];
	}
	return wf;
}

// Fetch the triangle formed by the facet vertices,
// making sure the facet orientation is kept (as in CGAL::Triangulation_3::triangle())
// return the vertex handles of the triangle
struct triangle_vhandles_t {
	vertex_handle_t verts[3];
	triangle_vhandles_t() {}
	triangle_vhandles_t(vertex_handle_t _v0, vertex_handle_t _v1, vertex_handle_t _v2)
		#ifdef _SUPPORT_CPP11
		: verts{_v0,_v1,_v2} {}
		#else
		{ verts[0] = _v0; verts[1] = _v1; verts[2] = _v2; }
		#endif
};
inline triangle_vhandles_t getTriangle(cell_handle_t cell, int i)
{
	ASSERT(i >= 0 && i <= 3);
	if ((i&1) == 0)
		return triangle_vhandles_t(
			cell->vertex((i+2)&3),
			cell->vertex((i+1)&3),
			cell->vertex((i+3)&3) );
	return triangle_vhandles_t(
		cell->vertex((i+1)&3),
		cell->vertex((i+2)&3),
		cell->vertex((i+3)&3) );
}

// Compute the angle between the plane containing the given facet and the cell's circumscribed sphere
// return cosines of the angle
float computePlaneSphereAngle(const delaunay_t& Tr, const facet_t& facet)
{
	// compute facet normal
	if (Tr.is_infinite(facet.first))
		return 1.f;
	const triangle_vhandles_t tri(getTriangle(facet.first, facet.second));
	const Point3f v0(CGAL2MVS<float>(tri.verts[0]->point()));
	const Point3f v1(CGAL2MVS<float>(tri.verts[1]->point()));
	const Point3f v2(CGAL2MVS<float>(tri.verts[2]->point()));
	const Point3f fn((v1-v0).cross(v2-v0));
		const float fnLenSq(normSq(fn));
		if (fnLenSq == 0.f)
			return 0.5f;

	// compute the co-tangent to the circumscribed sphere in one of the vertices
	#if CGAL_VERSION_NR < 1041101000
	const Point3f cc(CGAL2MVS<float>(facet.first->circumcenter(Tr.geom_traits())));
	#else
	struct Tools {
		static point_t circumcenter(const delaunay_t& Tr, const facet_t& facet) {
			return Tr.geom_traits().construct_circumcenter_3_object()(
				facet.first->vertex(0)->point(),
				facet.first->vertex(1)->point(),
				facet.first->vertex(2)->point(),
				facet.first->vertex(3)->point()
			);
		}
	};
	const Point3f cc(CGAL2MVS<float>(Tools::circumcenter(Tr, facet)));
	#endif
	const Point3f ct(cc-v0);
	const float ctLenSq(normSq(ct));
	if (ctLenSq == 0.f)
		return 0.5f;

	// compute the angle between the two vectors
	return CLAMP((fn.dot(ct))/SQRT(fnLenSq*ctLenSq), -1.f, 1.f);
}
} // namespace DELAUNAY

// First, iteratively create a Delaunay triangulation of the existing point-cloud by inserting point by point,
// iif the point to be inserted is not closer than distInsert pixels in at least one of its views to
// the projection of any of already inserted points.
// Next, the score is computed for all the edges of the directed graph composed of points as vertices.
// Finally, graph-cut algorithm is used to split the tetrahedrons in inside and outside,
// and the surface is such extracted.
bool Scene::ReconstructMesh(float distInsert, bool bUseFreeSpaceSupport, unsigned nItersFixNonManifold,
							float kSigma, float kQual, float kb,
							float kf, float kRel, float kAbs, float kOutl,
							float kInf
)
{
	using namespace DELAUNAY;
	ASSERT(!pointcloud.IsEmpty());
	mesh.Release();

	// create the Delaunay triangulation
	delaunay_t delaunay;
	std::vector<cell_info_t> infoCells;
	std::vector<camera_cell_t> camCells;
	std::vector<facet_t> hullFacets;
	{
		TD_TIMER_STARTD();

		std::vector<point_t> vertices(pointcloud.points.GetSize());
		std::vector<std::ptrdiff_t> indices(pointcloud.points.GetSize());
		// fetch points
		FOREACH(i, pointcloud.points) {
			const PointCloud::Point& X(pointcloud.points[i]);
			vertices[i] = point_t(X.x, X.y, X.z);
			indices[i] = i;
		}
		// sort vertices
		typedef CGAL::Spatial_sort_traits_adapter_3<delaunay_t::Geom_traits, point_t*> Search_traits;
		CGAL::spatial_sort(indices.begin(), indices.end(), Search_traits(&vertices[0], delaunay.geom_traits()));
		// insert vertices
		Util::Progress progress(_T("Points inserted"), indices.size());
		const float distInsertSq(SQUARE(distInsert));
		vertex_handle_t hint;
		delaunay_t::Locate_type lt;
		int li, lj;
		std::for_each(indices.cbegin(), indices.cend(), [&](size_t idx) {
			const point_t& p = vertices[idx];
			const PointCloud::Point& point = pointcloud.points[idx];
			const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
			ASSERT(!views.IsEmpty());
			if (hint == vertex_handle_t()) {
				// this is the first point,
				// insert it
				hint = delaunay.insert(p);
				ASSERT(hint != vertex_handle_t());
			} else
			if (distInsert <= 0) {
				// insert all points
				hint = delaunay.insert(p, hint);
				ASSERT(hint != vertex_handle_t());
			} else {
				// locate cell containing this point
				const cell_handle_t c(delaunay.locate(p, lt, li, lj, hint->cell()));
				if (lt == delaunay_t::VERTEX) {
					// duplicate point, nothing to insert,
					// just update its visibility info
					hint = c->vertex(li);
					ASSERT(hint != delaunay.infinite_vertex());
				} else {
					// locate the nearest vertex
					vertex_handle_t nearest;
					if (delaunay.dimension() < 3) {
						// use a brute-force algorithm if dimension < 3
						delaunay_t::Finite_vertices_iterator vit = delaunay.finite_vertices_begin();
						nearest = vit;
						++vit;
						adjacent_vertex_back_inserter_t inserter(delaunay, p, nearest);
						for (delaunay_t::Finite_vertices_iterator end = delaunay.finite_vertices_end(); vit != end; ++vit)
							inserter = vit;
					} else {
						// - start with the closest vertex from the located cell
						// - repeatedly take the nearest of its incident vertices if any
						// - if not, we're done
						ASSERT(c != cell_handle_t());
						nearest = delaunay.nearest_vertex_in_cell(p, c);
						while (true) {
							const vertex_handle_t v(nearest);
							delaunay.adjacent_vertices(nearest, adjacent_vertex_back_inserter_t(delaunay, p, nearest));
							if (v == nearest)
								break;
						}
					}
					ASSERT(nearest == delaunay.nearest_vertex(p, hint->cell()));
					hint = nearest;
					// check if point is far enough to all existing points
					FOREACHPTR(pViewID, views) {
						const Image& imageData = images[*pViewID];
						const Point3f pn(imageData.camera.ProjectPointP3(point));
						const Point3f pe(imageData.camera.ProjectPointP3(CGAL2MVS<float>(nearest->point())));
						if (!IsDepthSimilar(pn.z, pe.z) || normSq(Point2f(pn)-Point2f(pe)) > distInsertSq) {
							// point far enough to an existing point,
							// insert as a new point
							hint = delaunay.insert(p, lt, c, li, lj);
							ASSERT(hint != vertex_handle_t());
							break;
						}
					}
				}
			}
			// update point visibility info
			hint->info().InsertViews(pointcloud, idx);
			++progress;
		});
		progress.close();
		pointcloud.Release();
		// init cells weights and
		// loop over all cells and store the finite facet of the infinite cells
		const size_t numNodes(delaunay.number_of_cells());
		infoCells.resize(numNodes);
		memset(&infoCells[0], 0, sizeof(cell_info_t)*numNodes);
		cell_size_t ciID(0);
		for (delaunay_t::All_cells_iterator ci=delaunay.all_cells_begin(), eci=delaunay.all_cells_end(); ci!=eci; ++ci, ++ciID) {
			ci->info() = ciID;
			// skip the finite cells
			if (!delaunay.is_infinite(ci))
				continue;
			// find the finite face
			for (int f=0; f<4; ++f) {
				const facet_t facet(ci, f);
				if (!delaunay.is_infinite(facet)) {
					// store face
					hullFacets.push_back(facet);
					break;
				}
			}
		}
		// find all cells containing a camera
		camCells.resize(images.GetSize());
		FOREACH(i, images) {
			const Image& imageData = images[i];
			if (!imageData.IsValid())
				continue;
			const Camera& camera = imageData.camera;
			camera_cell_t& camCell = camCells[i];
			camCell.cell = delaunay.locate(MVS2CGAL(camera.C));
			ASSERT(camCell.cell != cell_handle_t());
			fetchCellFacets<CGAL::POSITIVE>(delaunay, hullFacets, camCell.cell, imageData, camCell.facets);
			// link all cells contained by the camera to the source
			for (const facet_t& f: camCell.facets)
				infoCells[f.first->info()].s = kInf;
		}

		DEBUG_EXTRA("Delaunay tetrahedralization completed: %u points -> %u vertices, %u (+%u) cells, %u (+%u) faces (%s)", indices.size(), delaunay.number_of_vertices(), delaunay.number_of_finite_cells(), delaunay.number_of_cells()-delaunay.number_of_finite_cells(), delaunay.number_of_finite_facets(), delaunay.number_of_facets()-delaunay.number_of_finite_facets(), TD_TIMER_GET_FMT().c_str());
	}

	// for every camera-point ray intersect it with the tetrahedrons and
	// add alpha_vis(point) to cell's directed edge in the graph
	{
		TD_TIMER_STARTD();

		// estimate the size of the smallest reconstructible object
		FloatArr distsSq(0, delaunay.number_of_edges());
		for (delaunay_t::Finite_edges_iterator ei=delaunay.finite_edges_begin(), eei=delaunay.finite_edges_end(); ei!=eei; ++ei) {
			const cell_handle_t& c(ei->first);
			distsSq.Insert(normSq(CGAL2MVS<float>(c->vertex(ei->second)->point()) - CGAL2MVS<float>(c->vertex(ei->third)->point())));
		}
		const float sigma(SQRT(distsSq.GetMedian())*kSigma);
		const float inv2SigmaSq(0.5f/(sigma*sigma));
		distsSq.Release();

		std::vector<facet_t> facets;

		// compute the weights for each edge
		{
		TD_TIMER_STARTD();
		Util::Progress progress(_T("Points weighted"), delaunay.number_of_vertices());
		#ifdef DELAUNAY_USE_OPENMP
		delaunay_t::Vertex_iterator vertexIter(delaunay.vertices_begin());
		const int64_t nVerts(delaunay.number_of_vertices()+1);
		#pragma omp parallel for private(facets)
		for (int64_t i=0; i<nVerts; ++i) {
			delaunay_t::Vertex_iterator vi;
			#pragma omp critical
			vi = vertexIter++;
		#else
		for (delaunay_t::Vertex_iterator vi=delaunay.vertices_begin(), vie=delaunay.vertices_end(); vi!=vie; ++vi) {
		#endif
			vert_info_t& vert(vi->info());
			if (vert.views.IsEmpty())
				continue;
			#ifdef DELAUNAY_WEAKSURF
			vert.AllocateInfo();
			#endif
			const point_t& p(vi->point());
			const Point3 pt(CGAL2MVS<REAL>(p));
			FOREACH(v, vert.views) {
				const typename vert_info_t::view_t view(vert.views[v]);
				const uint32_t imageID(view.idxView);
				const edge_cap_t alpha_vis(view.weight);
				const Image& imageData = images[imageID];
				ASSERT(imageData.IsValid());
				const Camera& camera = imageData.camera;
				const camera_cell_t& camCell = camCells[imageID];
				// compute the ray used to find point intersection
				const Point3 vecCamPoint(pt-camera.C);
				const REAL invLenCamPoint(REAL(1)/norm(vecCamPoint));
				intersection_t inter(pt, Point3(vecCamPoint*invLenCamPoint));
				// find faces intersected by the camera-point segment
				const segment_t segCamPoint(MVS2CGAL(camera.C), p);
				if (!intersect(delaunay, segCamPoint, camCell.facets, facets, inter))
					continue;
				do {
					// assign score, weighted by the distance from the point to the intersection
					const edge_cap_t w(alpha_vis*(1.f-EXP(-SQUARE((float)inter.dist)*inv2SigmaSq)));
					edge_cap_t& f(infoCells[inter.facet.first->info()].f[inter.facet.second]);
					#ifdef DELAUNAY_USE_OPENMP
					#pragma omp atomic
					#endif
					f += w;
				} while (intersect(delaunay, segCamPoint, facets, facets, inter));
				ASSERT(facets.empty() && inter.type == intersection_t::VERTEX && inter.v1 == vi);
				#ifdef DELAUNAY_WEAKSURF
				ASSERT(vert.viewsInfo[v].cell2Cam == NULL);
				vert.viewsInfo[v].cell2Cam = inter.facet.first;
				#endif
				// find faces intersected by the endpoint-point segment
				inter.dist = FLT_MAX; inter.bigger = false;
				const Point3 endPoint(pt+vecCamPoint*(invLenCamPoint*sigma));
				const segment_t segEndPoint(MVS2CGAL(endPoint), p);
				const cell_handle_t endCell(delaunay.locate(segEndPoint.source(), vi->cell()));
				ASSERT(endCell != cell_handle_t());
				fetchCellFacets<CGAL::NEGATIVE>(delaunay, hullFacets, endCell, imageData, facets);
				edge_cap_t& t(infoCells[endCell->info()].t);
				#ifdef DELAUNAY_USE_OPENMP
				#pragma omp atomic
				#endif
				t += alpha_vis;
				while (intersect(delaunay, segEndPoint, facets, facets, inter)) {
					// assign score, weighted by the distance from the point to the intersection
					const facet_t& mf(delaunay.mirror_facet(inter.facet));
					const edge_cap_t w(alpha_vis*(1.f-EXP(-SQUARE((float)inter.dist)*inv2SigmaSq)));
					edge_cap_t& f(infoCells[mf.first->info()].f[mf.second]);
					#ifdef DELAUNAY_USE_OPENMP
					#pragma omp atomic
					#endif
					f += w;
				}
				ASSERT(facets.empty() && inter.type == intersection_t::VERTEX && inter.v1 == vi);
				#ifdef DELAUNAY_WEAKSURF
				ASSERT(vert.viewsInfo[v].cell2End == NULL);
				vert.viewsInfo[v].cell2End = inter.facet.first;
				#endif
			}
			++progress;
		}
		progress.close();
		DEBUG_ULTIMATE("\tweighting completed in %s", TD_TIMER_GET_FMT().c_str());
		}
		camCells.clear();

		#ifdef DELAUNAY_WEAKSURF
		// enforce t-edges for each point-camera pair with free-space support weights
		if (bUseFreeSpaceSupport) {
		TD_TIMER_STARTD();
		#ifdef DELAUNAY_USE_OPENMP
		delaunay_t::Vertex_iterator vertexIter(delaunay.vertices_begin());
		const int64_t nVerts(delaunay.number_of_vertices()+1);
		#pragma omp parallel for private(facets)
		for (int64_t i=0; i<nVerts; ++i) {
			delaunay_t::Vertex_iterator vi;
			#pragma omp critical
			vi = vertexIter++;
		#else
		for (delaunay_t::Vertex_iterator vi=delaunay.vertices_begin(), vie=delaunay.vertices_end(); vi!=vie; ++vi) {
		#endif
			const vert_info_t& vert(vi->info());
			if (vert.views.IsEmpty())
				continue;
			const point_t& p(vi->point());
			const Point3f pt(CGAL2MVS<float>(p));
			FOREACH(v, vert.views) {
				const uint32_t imageID(vert.views[(vert_info_t::view_vec_t::IDX)v]);
				const Image& imageData = images[imageID];
				ASSERT(imageData.IsValid());
				const Camera& camera = imageData.camera;
				// compute the ray used to find point intersection
				const Point3f vecCamPoint(pt-Cast<float>(camera.C));
				const float invLenCamPoint(1.f/norm(vecCamPoint));
				// find faces intersected by the point-camera segment and keep the max free-space support score
				const Point3f bgnPoint(pt-vecCamPoint*(invLenCamPoint*sigma*kf));
				const segment_t segPointBgn(p, MVS2CGAL(bgnPoint));
				intersection_t inter;
				if (!intersectFace(delaunay, segPointBgn, vi, vert.viewsInfo[v].cell2Cam, facets, inter))
					continue;
				edge_cap_t beta(0);
				do {
					const edge_cap_t fs(freeSpaceSupport(delaunay, infoCells, inter.facet.first));
					if (beta < fs)
						beta = fs;
				} while (intersectFace(delaunay, segPointBgn, facets, facets, inter));
				// find faces intersected by the point-endpoint segment
				const Point3f endPoint(pt+vecCamPoint*(invLenCamPoint*sigma*kb));
				const segment_t segPointEnd(p, MVS2CGAL(endPoint));
				if (!intersectFace(delaunay, segPointEnd, vi, vert.viewsInfo[v].cell2End, facets, inter))
					continue;
				edge_cap_t gammaMin(FLT_MAX), gammaMax(0);
				do {
					const edge_cap_t fs(freeSpaceSupport(delaunay, infoCells, inter.facet.first));
					if (gammaMin > fs)
						gammaMin = fs;
					if (gammaMax < fs)
						gammaMax = fs;
				} while (intersectFace(delaunay, segPointEnd, facets, facets, inter));
				const edge_cap_t gamma((gammaMin+gammaMax)*0.5f);
				// if the point can be considered an interface point,
				// enforce the t-edge weight of the end cell
				const edge_cap_t epsAbs(beta-gamma);
				const edge_cap_t epsRel(gamma/beta);
				if (epsRel < kRel && epsAbs > kAbs && gamma < kOutl) {
					edge_cap_t& t(infoCells[inter.ncell->info()].t);
					#ifdef DELAUNAY_USE_OPENMP
					#pragma omp atomic
					#endif
					t *= epsAbs;
				}
			}
		}
		DEBUG_ULTIMATE("\tt-edge reinforcement completed in %s", TD_TIMER_GET_FMT().c_str());
		}
		#endif

		DEBUG_EXTRA("Delaunay tetrahedras weighting completed: %u cells, %u faces (%s)", delaunay.number_of_cells(), delaunay.number_of_facets(), TD_TIMER_GET_FMT().c_str());
	}

	// run graph-cut and extract the mesh
	{
		TD_TIMER_STARTD();

		// create graph
		MaxFlow<cell_size_t,edge_cap_t> graph(delaunay.number_of_cells());
		// set weights
		for (delaunay_t::All_cells_iterator ci=delaunay.all_cells_begin(), ce=delaunay.all_cells_end(); ci!=ce; ++ci) {
			const cell_size_t ciID(ci->info());
			const cell_info_t& ciInfo(infoCells[ciID]);
			graph.AddNode(ciID, ciInfo.s, ciInfo.t);
			for (int i=0; i<4; ++i) {
				const cell_handle_t cj(ci->neighbor(i));
				const cell_size_t cjID(cj->info());
				if (cjID < ciID) continue;
				const cell_info_t& cjInfo(infoCells[cjID]);
				const int j(cj->index(ci));
				const edge_cap_t q((1.f - MINF(computePlaneSphereAngle(delaunay, facet_t(ci,i)), computePlaneSphereAngle(delaunay, facet_t(cj,j))))*kQual);
				graph.AddEdge(ciID, cjID, ciInfo.f[i]+q, cjInfo.f[j]+q);
			}
		}
		infoCells.clear();
		// find graph-cut solution
		const float maxflow(graph.ComputeMaxFlow());
		// extract surface formed by the facets between inside/outside cells
		const size_t nEstimatedNumVerts(delaunay.number_of_vertices());
		std::unordered_map<void*,Mesh::VIndex> mapVertices;
		#if defined(_MSC_VER) && (_MSC_VER > 1600)
		mapVertices.reserve(nEstimatedNumVerts);
		#endif
		mesh.vertices.Reserve((Mesh::VIndex)nEstimatedNumVerts);
		mesh.faces.Reserve((Mesh::FIndex)nEstimatedNumVerts*2);
		for (delaunay_t::All_cells_iterator ci=delaunay.all_cells_begin(), ce=delaunay.all_cells_end(); ci!=ce; ++ci) {
			const cell_size_t ciID(ci->info());
			for (int i=0; i<4; ++i) {
				if (delaunay.is_infinite(ci, i)) continue;
				const cell_handle_t cj(ci->neighbor(i));
				const cell_size_t cjID(cj->info());
				if (ciID < cjID) continue;
				const bool ciType(graph.IsNodeOnSrcSide(ciID));
				if (ciType == graph.IsNodeOnSrcSide(cjID)) continue;
				Mesh::Face& face = mesh.faces.AddEmpty();
				const triangle_vhandles_t tri(getTriangle(ci, i));
				for (int v=0; v<3; ++v) {
					const vertex_handle_t vh(tri.verts[v]);
					ASSERT(vh->point() == delaunay.triangle(ci,i)[v]);
					const auto pairItID(mapVertices.insert(std::make_pair(vh.for_compact_container(), (Mesh::VIndex)mesh.vertices.GetSize())));
					if (pairItID.second)
						mesh.vertices.Insert(CGAL2MVS<Mesh::Vertex::Type>(vh->point()));
					ASSERT(pairItID.first->second < mesh.vertices.GetSize());
					face[v] = pairItID.first->second;
				}
				// correct face orientation
				if (!ciType)
					std::swap(face[0], face[2]);
			}
		}
		delaunay.clear();

		DEBUG_EXTRA("Delaunay tetrahedras graph-cut completed (%g flow): %u vertices, %u faces (%s)", maxflow, mesh.vertices.GetSize(), mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());
	}

	// fix non-manifold vertices and edges
	for (unsigned i=0; i<nItersFixNonManifold; ++i)
		if (!mesh.FixNonManifold())
			break;
	return true;
}
/*----------------------------------------------------------------*/
