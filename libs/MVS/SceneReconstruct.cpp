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
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_triangle_primitive_3.h>
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

#pragma push_macro("VERBOSE")
#undef VERBOSE
#define VERBOSE(...) LOG(lt, __VA_ARGS__)


// S T R U C T S ///////////////////////////////////////////////////

DEFINE_LOG_NAME(lt, _T("ScnRecnt"));

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

// construct the mesh out of the dense point-cloud using Delaunay tetrahedralization & graph-cut method
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
			// pointWeights holds the plain [0,1] per-view confidence (see SceneDensify fusion), i.e.
			// the expected value of the constant-weight vote of 1: dimensionless and independent of
			// the scene's length unit, so it can feed the graph-cut constants (kb, kf, kRel, kAbs,
			// kOutl, tuned for the uniform-weight regime) directly, with no normalization step;
			// a point-cloud without weights votes 1 per view
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
	viewsInfo = new view_info_t[views.size()];
	#ifndef _RELEASE
	memset(reinterpret_cast<void*>(viewsInfo), 0, sizeof(view_info_t)*views.size());
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

// |log2(median Delaunay edge)| past which the canonical rescale engages; inside the band the
// orientation predicate below has orders of magnitude of headroom either way
constexpr double kLog2CanonicalBand = 10;
// Canonical coordinate rescale of the triangulation (ReconstructMeshParams::bCanonicalRescale).
// orientation() tests an unnormalized determinant, which grows as the cube of the local edge
// length, against a fixed absolute epsilon, so the predicate is calibrated only while the scene's
// median Delaunay edge stays near one unit: far below it every call answers COPLANAR and the ray
// walks collapse, far above it the epsilon is inert. A uniform power-of-two factor moves the median
// edge back into that band. Power of two matters twice: multiplying by it is exact in IEEE
// arithmetic (it shifts the exponent, the mantissa is untouched), so the working coordinates and
// the inverse applied at extraction round-trip bit-for-bit; and the kernel's exact predicates
// answer identically at any scale, so every cell and vertex handle survives and nothing is
// retriangulated. This repairs the predicate only: a scene whose geometry the float storage of
// PointCloud::Point already quantized away needs centering at load time, before this code runs.
struct coord_rescale_t {
	double scale;    // scene space -> working space
	double invScale; // working space -> scene space
	int exponent;    // log2(scale), reported when the rescale engages
	bool bEnabled;   // false keeps every consumer below on its untouched, unscaled path
	inline coord_rescale_t() : scale(1), invScale(1), exponent(0), bEnabled(false) {}
	// decide the factor from the measured median edge length
	inline void Setup(float medianEdge) {
		ASSERT(!bEnabled);
		ASSERT(ISFINITE(medianEdge) && medianEdge > 0); // validated where measured
		const double log2Edge(std::log2((double)medianEdge));
		if (ABS(log2Edge) <= kLog2CanonicalBand)
			return;
		exponent = -(int)std::lround(log2Edge);
		scale = std::ldexp(1.0, exponent);
		invScale = std::ldexp(1.0, -exponent);
		bEnabled = true;
	}
	// unconditional, for the single loop that scales the triangulation in place
	inline point_t Scaled(const point_t& p) const {
		ASSERT(bEnabled);
		return point_t(p.x()*scale, p.y()*scale, p.z()*scale);
	}
	// every other conversion branches, so the disabled path executes the original code exactly
	inline Point3 ToWorking(const Point3& p) const { return bEnabled ? Point3(p*scale) : p; }
	inline point_t ToWorld(const point_t& p) const { return bEnabled ? point_t(p.x()*invScale, p.y()*invScale, p.z()*invScale) : p; }
};

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
// Disable FP contraction (FMA) for this geometric predicate so the sign of the
// determinant is reproducible across architectures and compilers: a fused
// multiply-add rounds once instead of twice and can flip the sign near the
// epsilon threshold
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("-ffp-contract=off")
#endif
static inline int orientation(const point_t& a, const point_t& b, const point_t& c, const point_t& p)
{
	#if defined(__clang__)
	#pragma clang fp contract(off)
	#endif
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
#if defined(__GNUC__) && !defined(__clang__)
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
void fetchCellFacets(const delaunay_t& Tr, const std::vector<facet_t>& hullFacets, const cell_handle_t& cell, const Image& imageData, const coord_rescale_t& rescale, std::vector<facet_t>& facets)
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
	const TFrustum<REAL,4> frustum(imageData.camera.P, imageData.width, imageData.height, 0, 1);
	// loop over all cells
	// the orientation test runs in the working space, which is the whole point of the rescale,
	// while the frustum keeps the camera's own space and the facet bounds are mapped back into
	// it: the round-trip is exact, so the culling verdict is the same at any scale
	const point_t ptOrigin(MVS2CGAL(rescale.ToWorking(imageData.camera.C)));
	for (const facet_t& face: hullFacets) {
		// add face if visible
		const triangle_t verts(Tr.triangle(face));
		if (orientation(verts[0], verts[1], verts[2], ptOrigin) != FacetOrientation)
			continue;
		AABB3 ab(CGAL2MVS<REAL>(rescale.ToWorld(verts[0])));
		for (int i=1; i<3; ++i)
			ab.Insert(CGAL2MVS<REAL>(rescale.ToWorld(verts[i])));
		if (frustum.Classify(ab) == CULLED)
			continue;
		facets.push_back(face);
	}
}


// aggregate ray-walk accounting; observed only, never fed back into the reconstruction
struct walk_stats_t {
	uint64_t nSteps; // facet/edge/vertex steps accepted by intersect()
	uint64_t nBadEnd; // intersect() gave up with the segment not consumed
	uint64_t nCamRayDropped; // camera-side walk failed on its first step, the whole ray is discarded
	uint64_t nCamWalkAborted; // camera-side walk did not end on its own vertex, so cell2Cam is wrong
	uint64_t nEndWalkAborted; // end-point-side walk did not end on its own vertex, so cell2End is wrong
	uint64_t padCacheLine[3]; // keeps two neighbor slots of the pool below on separate cache lines

	inline walk_stats_t& operator += (const walk_stats_t& r) {
		nSteps += r.nSteps;
		nBadEnd += r.nBadEnd;
		nCamRayDropped += r.nCamRayDropped;
		nCamWalkAborted += r.nCamWalkAborted;
		nEndWalkAborted += r.nEndWalkAborted;
		return *this;
	}
};
// one slot per worker thread, so the increments need no synchronization;
// the slots are summed only after both weighting loops have completed.
// The pool is process-wide and reset at the start of every reconstruction, so consecutive
// reconstructions each report their own counts but two running at the same time would share
// the slots: ReconstructMesh is reentrant across calls, not across concurrent callers
static std::vector<walk_stats_t> g_walkStatsPool;
static inline walk_stats_t& GetWalkStats() {
	#ifdef DELAUNAY_USE_OPENMP
	const size_t idx((size_t)omp_get_thread_num());
	ASSERT(idx < g_walkStatsPool.size());
	return g_walkStatsPool[idx];
	#else
	ASSERT(!g_walkStatsPool.empty());
	return g_walkStatsPool.front();
	#endif
}
static inline void ResetWalkStats() {
	#ifdef DELAUNAY_USE_OPENMP
	g_walkStatsPool.assign((size_t)omp_get_max_threads(), walk_stats_t());
	#else
	g_walkStatsPool.assign(1, walk_stats_t());
	#endif
}
static inline walk_stats_t GetTotalWalkStats() {
	walk_stats_t total{};
	for (const walk_stats_t& stats: g_walkStatsPool)
		total += stats;
	return total;
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
//  stats [in,out] : the calling thread's accounting slot, passed in because this runs once per
//    walk step and looking the slot up here would cost an omp_get_thread_num() call each time
// return false if no intersection found and the end of the segment was not reached
bool intersect(const delaunay_t& Tr, const segment_t& seg, const std::vector<facet_t>& in_facets, std::vector<facet_t>& out_facets, intersection_t& inter, walk_stats_t& stats)
{
	ASSERT(!in_facets.empty());
	static const int facet_vertex_order[] = {2,1,3,2,2,3,0,2,0,3,1,0,0,1,2,0};
	int coplanar[3];
	const REAL prevDist(inter.dist);
	for (const facet_t& in_facet: in_facets) {
		ASSERT(!Tr.is_infinite(in_facet));
		const int nb_coplanar(intersect(Tr.triangle(in_facet), seg, coplanar));
		if (nb_coplanar >= 0) {
			if (nb_coplanar == 3) {
				// coplanar with 3 edges = tangent: the segment travels in the facet's
				// supporting plane, so no crossing distance exists; give up
				break;
			}
			// skip this cell if the intersection is not in the desired direction
			const REAL interDist(inter.ray.IntersectsDist(getFacetPlane(in_facet)));
			ASSERT(ISFINITE(interDist)); // the exact test above says the segment straddles this plane
			if ((interDist > prevDist) != inter.bigger) {
				continue;
			}
			// vertices of facet i: j = 4 * i, vertices = facet_vertex_order[j,j+1,j+2] negative orientation
			inter.facet = in_facet;
			inter.dist = interDist;
			++stats.nSteps;
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
			ASSERT("should not happen" == NULL);
		}
	}
	// Bad end: no intersection found and we are not at the end of the segment (very rarely, but it happens)!
	++stats.nBadEnd;
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
bool Scene::ReconstructMesh(float distInsert, bool bUseFreeSpaceSupport, bool bUseOnlyROI,
							float kSigma, float kQual, const ReconstructMeshParams& params,
							float kb, float kf, float kRel, float kAbs, float kOutl,
							float kInf)
{
	using namespace DELAUNAY;
	ASSERT(!pointcloud.IsEmpty());
	mesh.Release();

	// create the Delaunay triangulation
	delaunay_t delaunay;
	std::vector<cell_info_t> infoCells;
	std::vector<camera_cell_t> camCells;
	std::vector<facet_t> hullFacets;
	// median length over all finite Delaunay edges, measured once the triangulation is complete:
	// the reference the canonical rescale is decided from and the base of sigma further down.
	// Both live in the working space, so a rescaled scene keeps every distance comparison
	// downstream on the same footing
	float medianEdge(0);
	coord_rescale_t rescale;
	{
		TD_TIMER_STARTD();

		std::vector<point_t> vertices(pointcloud.points.GetSize());
		std::vector<std::ptrdiff_t> indices;
		indices.reserve(pointcloud.points.GetSize());
		// fetch points
		if (bUseOnlyROI && !IsBounded())
			bUseOnlyROI = false;
		FOREACH(i, pointcloud.points) {
			const PointCloud::Point& X(pointcloud.points[i]);
			if (bUseOnlyROI && !obb.Intersects(X))
				continue;
			vertices[i] = point_t(X.x, X.y, X.z);
			indices.emplace_back(i);
		}
		if (indices.empty()) {
			VERBOSE("error: no points available for Delaunay reconstruction");
			return false;
		}
		// sort vertices
		typedef CGAL::Spatial_sort_traits_adapter_3<delaunay_t::Geom_traits, point_t*> Search_traits;
		CGAL::spatial_sort(indices.begin(), indices.end(), Search_traits(vertices.data(), delaunay.geom_traits()));
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
			hint->info().InsertViews(pointcloud, (PointCloud::Index)idx);
			++progress;
		});
		progress.close();
		pointcloud.Release();
		if (delaunay.dimension() < 3) {
			VERBOSE("error: too few or degenerate points for Delaunay reconstruction (dimension %d)", delaunay.dimension());
			return false;
		}
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
		// estimate the size of the smallest reconstructible object: the median of the squared
		// finite edge lengths, square-rooted (median commutes with the square root, so this is
		// the median length itself and the long-edge tail cannot pull it)
		{
			FloatArr distsSq(0, delaunay.number_of_edges());
			for (delaunay_t::Finite_edges_iterator ei=delaunay.finite_edges_begin(), eei=delaunay.finite_edges_end(); ei!=eei; ++ei) {
				const cell_handle_t& c(ei->first);
				distsSq.Insert(normSq(CGAL2MVS<float>(c->vertex(ei->second)->point()) - CGAL2MVS<float>(c->vertex(ei->third)->point())));
			}
			medianEdge = SQRT(distsSq.GetMedian());
		}
		// the boundary where the measurement of the input data becomes an internal invariant:
		// everything downstream (canonical rescale, sigma) relies on a usable scale, so a cloud
		// whose median edge cannot be represented in float is rejected here, once
		if (!ISFINITE(medianEdge) || medianEdge <= 0) {
			VERBOSE("error: degenerate point-cloud scale (median Delaunay edge %g)", medianEdge);
			return false;
		}
		// canonical rescale: everything from here on - the camera cells located below, both
		// weighting loops and sigma itself - lives in the working space, and
		// only the extracted mesh vertices are mapped back
		if (params.bCanonicalRescale) {
			rescale.Setup(medianEdge);
			if (rescale.bEnabled) {
				for (delaunay_t::Finite_vertices_iterator vit=delaunay.finite_vertices_begin(), vite=delaunay.finite_vertices_end(); vit!=vite; ++vit)
					vit->set_point(rescale.Scaled(vit->point()));
				const float medianEdgeScene(medianEdge);
				medianEdge = (float)(medianEdge*rescale.scale);
				DEBUG_EXTRA("Canonical rescale engaged: median Delaunay edge %g -> %g (scale 2^%d)", medianEdgeScene, medianEdge, rescale.exponent);
			} else
				DEBUG_ULTIMATE("\tcanonical rescale not needed: median Delaunay edge %g inside [2^-%g, 2^%g]", medianEdge, kLog2CanonicalBand, kLog2CanonicalBand);
		}
		// find all cells containing a camera
		camCells.resize(images.GetSize());
		FOREACH(i, images) {
			const Image& imageData = images[i];
			if (!imageData.IsValid())
				continue;
			const Camera& camera = imageData.camera;
			camera_cell_t& camCell = camCells[i];
			camCell.cell = delaunay.locate(MVS2CGAL(rescale.ToWorking(camera.C)));
			ASSERT(camCell.cell != cell_handle_t());
			fetchCellFacets<CGAL::POSITIVE>(delaunay, hullFacets, camCell.cell, imageData, rescale, camCell.facets);
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

		// ReconstructMesh can run more than once per process, so the accounting starts clean here
		ResetWalkStats();
		// scene coordinate magnitude, reported next to the ray-walk accounting below; measured in
		// the working space, so both it and the L it is printed with describe the space the walks
		// actually run in rather than the scene's own units
		AABB3 bbVerts(true);
		for (delaunay_t::Finite_vertices_iterator vit=delaunay.finite_vertices_begin(), vite=delaunay.finite_vertices_end(); vit!=vite; ++vit)
			bbVerts.InsertFull(CGAL2MVS<REAL>(vit->point()));
		const REAL sceneMagnitude(bbVerts.GetCenter().norm());

		// the size of the smallest reconstructible object, from the median edge measured with
		// the triangulation above and already expressed in the working space
		const float sigma(medianEdge*kSigma);
		const float inv2SigmaSq(0.5f/(sigma*sigma));

		// capacity reserved once per thread for the ray-walk facet front, which stays
		// small: at most the four facets of a cell, or the facets incident to a vertex
		constexpr size_t kFacetsReserve(64);

		// vertices weighted by the two loops below, collected once and reused by both.
		// The vertex range spans the infinite vertex as well, but neither it nor any
		// view-less vertex contributes weight, so both are filtered out here rather
		// than tested inside the loops; this also gives the loops a random-access
		// index, replacing the locked iterator handoff that used to serialize them.
		std::vector<vertex_handle_t> vertexHandles;
		vertexHandles.reserve(delaunay.number_of_vertices());
		for (delaunay_t::Vertex_iterator vi=delaunay.vertices_begin(), vie=delaunay.vertices_end(); vi!=vie; ++vi)
			if (!vi->info().views.IsEmpty())
				vertexHandles.push_back(vi);
		const int64_t nVerts((int64_t)vertexHandles.size());

		// per-vertex uncertainty: the global sigma is kSigma times the median length over all
		// finite edges, so its local counterpart is the same statistic restricted to the edges
		// incident to the vertex, clamped to [0.25,4] x global to keep the walks bounded;
		// indexed like vertexHandles, and allocated only by this arm
		const bool bAdaptiveSigma(params.bAdaptiveSigma);
		std::vector<float> sigmaVert;
		if (bAdaptiveSigma) {
			TD_TIMER_STARTD();
			const float sigmaVertMin(sigma*0.25f), sigmaVertMax(sigma*4.f);
			sigmaVert.resize((size_t)nVerts);
			#ifdef DELAUNAY_USE_OPENMP
			#pragma omp parallel
			{
			// the threadsafe traversal keeps its visited set thread-local, while the plain
			// finite_incident_edges() marks the shared cell state the ray-walks also read
			std::vector<edge_t> edges;
			FloatArr edgeDistsSq;
			#pragma omp for schedule(dynamic)
			for (int64_t i=0; i<nVerts; ++i) {
			#else
			std::vector<edge_t> edges;
			FloatArr edgeDistsSq;
			for (int64_t i=0; i<nVerts; ++i) {
			#endif
				const vertex_handle_t vi(vertexHandles[(size_t)i]);
				edges.clear();
				delaunay.finite_incident_edges_threadsafe(vi, std::back_inserter(edges));
				edgeDistsSq.Empty();
				for (const edge_t& e: edges) {
					const cell_handle_t& c(e.first);
					edgeDistsSq.Insert(normSq(CGAL2MVS<float>(c->vertex(e.second)->point()) - CGAL2MVS<float>(c->vertex(e.third)->point())));
				}
				// every finite vertex of a 3D triangulation has at least one finite incident edge
				ASSERT(!edgeDistsSq.IsEmpty());
				sigmaVert[(size_t)i] = CLAMP(SQRT(edgeDistsSq.GetMedian())*kSigma, sigmaVertMin, sigmaVertMax);
			}
			#ifdef DELAUNAY_USE_OPENMP
			} // omp parallel
			#endif
			// spread reported in units of the global sigma, so a uniform-scale scene reads ~1
			FloatArr sigmaRatios(0, (FloatArr::IDX)nVerts);
			const float invSigma(1.f/sigma);
			float ratioMin(FLT_MAX), ratioMax(0.f);
			size_t nClampedLow(0), nClampedHigh(0);
			for (int64_t i=0; i<nVerts; ++i) {
				const float sigmaV(sigmaVert[(size_t)i]);
				if (sigmaV <= sigmaVertMin)
					++nClampedLow;
				else if (sigmaV >= sigmaVertMax)
					++nClampedHigh;
				const float ratio(sigmaV*invSigma);
				if (ratioMin > ratio)
					ratioMin = ratio;
				if (ratioMax < ratio)
					ratioMax = ratio;
				sigmaRatios.Insert(ratio);
			}
			DEBUG_EXTRA("Adaptive sigma: %lld vertices, sigma_v/sigma min %.3f, median %.3f, max %.3f, clamped low %.2f%%, high %.2f%% (%s)",
				(long long)nVerts, ratioMin, sigmaRatios.GetMedian(), ratioMax,
				100.f*(float)nClampedLow/(float)nVerts, 100.f*(float)nClampedHigh/(float)nVerts, TD_TIMER_GET_FMT().c_str());
		}

		// compute the weights for each edge
		{
		TD_TIMER_STARTD();
		Util::Progress progress(_T("Points weighted"), delaunay.number_of_vertices());
		#ifdef DELAUNAY_USE_OPENMP
		#pragma omp parallel
		{
		std::vector<facet_t> facets;
		facets.reserve(kFacetsReserve);
		#pragma omp for schedule(dynamic)
		for (int64_t i=0; i<nVerts; ++i) {
			const vertex_handle_t vi(vertexHandles[(size_t)i]);
		#else
		std::vector<facet_t> facets;
		facets.reserve(kFacetsReserve);
		for (int64_t i=0; i<nVerts; ++i) {
			const vertex_handle_t vi(vertexHandles[(size_t)i]);
		#endif
			vert_info_t& vert(vi->info());
			#ifdef DELAUNAY_WEAKSURF
			if (bUseFreeSpaceSupport)
				vert.AllocateInfo();
			#endif
			const point_t& p(vi->point());
			const Point3 pt(CGAL2MVS<REAL>(p));
			// the point's own uncertainty, scaling the soft-visibility fall-off and the
			// end-cell offset below; both reduce to the global sigma when the arm is off
			const float sigmaV(bAdaptiveSigma ? sigmaVert[(size_t)i] : sigma);
			const float inv2SigmaSqV(bAdaptiveSigma ? 0.5f/(sigmaV*sigmaV) : inv2SigmaSq);
			walk_stats_t& stats(GetWalkStats());
			FOREACH(v, vert.views) {
				const typename vert_info_t::view_t view(vert.views[v]);
				const uint32_t imageID(view.idxView);
				const edge_cap_t alpha_vis(view.weight);
				const Image& imageData = images[imageID];
				ASSERT(imageData.IsValid());
				const Camera& camera = imageData.camera;
				const camera_cell_t& camCell = camCells[imageID];
				// compute the ray used to find point intersection
				const Point3 camC(rescale.ToWorking(camera.C));
				const Point3 vecCamPoint(pt-camC);
				const REAL invLenCamPoint(REAL(1)/norm(vecCamPoint));
				intersection_t inter(pt, Point3(vecCamPoint*invLenCamPoint));
				// find faces intersected by the camera-point segment
				const segment_t segCamPoint(MVS2CGAL(camC), p);
				if (!intersect(delaunay, segCamPoint, camCell.facets, facets, inter, stats)) {
					++stats.nCamRayDropped;
					continue;
				}
				do {
					// assign score, weighted by the distance from the point to the intersection
					const edge_cap_t w(alpha_vis*(1.f-EXP(-SQUARE((float)inter.dist)*inv2SigmaSqV)));
					edge_cap_t& f(infoCells[inter.facet.first->info()].f[inter.facet.second]);
					#ifdef DELAUNAY_USE_OPENMP
					#pragma omp atomic
					#endif
					f += w;
				} while (intersect(delaunay, segCamPoint, facets, facets, inter, stats));
				const bool bCamWalkOK(facets.empty() && inter.type == intersection_t::VERTEX && inter.v1 == vi);
				ASSERT(bCamWalkOK);
				if (!bCamWalkOK)
					++stats.nCamWalkAborted;
				#ifdef DELAUNAY_WEAKSURF
				if (bUseFreeSpaceSupport) {
					ASSERT(vert.viewsInfo[v].cell2Cam == NULL);
					vert.viewsInfo[v].cell2Cam = inter.facet.first;
				}
				#endif
				// find faces intersected by the endpoint-point segment
				inter.dist = FLT_MAX; inter.bigger = false;
				const Point3 endPoint(pt+vecCamPoint*(invLenCamPoint*sigmaV));
				const segment_t segEndPoint(MVS2CGAL(endPoint), p);
				const cell_handle_t endCell(delaunay.locate(segEndPoint.source(), vi->cell()));
				ASSERT(endCell != cell_handle_t());
				fetchCellFacets<CGAL::NEGATIVE>(delaunay, hullFacets, endCell, imageData, rescale, facets);
				edge_cap_t& t(infoCells[endCell->info()].t);
				#ifdef DELAUNAY_USE_OPENMP
				#pragma omp atomic
				#endif
				t += alpha_vis;
				while (intersect(delaunay, segEndPoint, facets, facets, inter, stats)) {
					// assign score, weighted by the distance from the point to the intersection
					const facet_t& mf(delaunay.mirror_facet(inter.facet));
					const edge_cap_t w(alpha_vis*(1.f-EXP(-SQUARE((float)inter.dist)*inv2SigmaSqV)));
					edge_cap_t& f(infoCells[mf.first->info()].f[mf.second]);
					#ifdef DELAUNAY_USE_OPENMP
					#pragma omp atomic
					#endif
					f += w;
				}
				const bool bEndWalkOK(facets.empty() && inter.type == intersection_t::VERTEX && inter.v1 == vi);
				ASSERT(bEndWalkOK);
				if (!bEndWalkOK)
					++stats.nEndWalkAborted;
				#ifdef DELAUNAY_WEAKSURF
				if (bUseFreeSpaceSupport) {
					ASSERT(vert.viewsInfo[v].cell2End == NULL);
					vert.viewsInfo[v].cell2End = inter.facet.first;
				}
				#endif
			}
			++progress;
		}
		#ifdef DELAUNAY_USE_OPENMP
		} // omp parallel
		#endif
		progress.close();
		DEBUG_ULTIMATE("\tweighting completed in %s", TD_TIMER_GET_FMT().c_str());
		}
		camCells.clear();

		#ifdef DELAUNAY_WEAKSURF
		// enforce t-edges for each point-camera pair with free-space support weights
		if (bUseFreeSpaceSupport) {
		TD_TIMER_STARTD();
		#ifdef DELAUNAY_USE_OPENMP
		#pragma omp parallel
		{
		std::vector<facet_t> facets;
		facets.reserve(kFacetsReserve);
		#pragma omp for schedule(dynamic)
		for (int64_t i=0; i<nVerts; ++i) {
			const vertex_handle_t vi(vertexHandles[(size_t)i]);
		#else
		std::vector<facet_t> facets;
		facets.reserve(kFacetsReserve);
		for (int64_t i=0; i<nVerts; ++i) {
			const vertex_handle_t vi(vertexHandles[(size_t)i]);
		#endif
			const vert_info_t& vert(vi->info());
			const point_t& p(vi->point());
			const Point3f pt(CGAL2MVS<float>(p));
			// same per-point uncertainty as the weighting loop, here sizing both search windows
			const float sigmaV(bAdaptiveSigma ? sigmaVert[(size_t)i] : sigma);
			FOREACH(v, vert.views) {
				const uint32_t imageID(vert.views[(vert_info_t::view_vec_t::IDX)v]);
				const Image& imageData = images[imageID];
				ASSERT(imageData.IsValid());
				const Camera& camera = imageData.camera;
				// compute the ray used to find point intersection
				const Point3f vecCamPoint(pt-Cast<float>(rescale.ToWorking(camera.C)));
				const float invLenCamPoint(1.f/norm(vecCamPoint));
				// find faces intersected by the point-camera segment and keep the max free-space support score
				const Point3f bgnPoint(pt-vecCamPoint*(invLenCamPoint*sigmaV*kf));
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
				const Point3f endPoint(pt+vecCamPoint*(invLenCamPoint*sigmaV*kb));
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
					// multiplied once per firing (vertex, view) pair; a zero t stays zero by
					// design - enforcing on cells no visibility vote ever reached collapses
					// thin structures, so the no-op is protective, not a defect
					edge_cap_t& t(infoCells[inter.ncell->info()].t);
					#ifdef DELAUNAY_USE_OPENMP
					#pragma omp atomic
					#endif
					t *= epsAbs;
				}
			}
		}
		#ifdef DELAUNAY_USE_OPENMP
		} // omp parallel
		#endif
		DEBUG_ULTIMATE("\tt-edge reinforcement completed in %s", TD_TIMER_GET_FMT().c_str());
		}
		#endif

		const walk_stats_t walkStats(GetTotalWalkStats());
		if (walkStats.nBadEnd)
			DEBUG_EXTRA("warning: %llu ray-walks ended badly (%.4f%% of %llu steps), %llu rays dropped, %llu walks aborted (M=%g L=%g)",
				(unsigned long long)walkStats.nBadEnd, 100.0*(double)walkStats.nBadEnd/(double)MAXF(walkStats.nSteps, uint64_t(1)),
				(unsigned long long)walkStats.nSteps, (unsigned long long)walkStats.nCamRayDropped,
				(unsigned long long)(walkStats.nCamWalkAborted+walkStats.nEndWalkAborted),
				sceneMagnitude, sigma/kSigma);
		DEBUG_EXTRA("Delaunay tetrahedras weighting completed: %u cells, %u faces (%s)", delaunay.number_of_cells(), delaunay.number_of_facets(), TD_TIMER_GET_FMT().c_str());
	}

	// run graph-cut and extract the mesh
	{
		TD_TIMER_STARTD();

		// create graph
		MaxFlow<cell_size_t,edge_cap_t> graph(delaunay.number_of_cells());
		// set weights
		constexpr edge_cap_t maxCap(3.402823466e+34f/*FLT_MAX*0.0001f*/);
		for (delaunay_t::All_cells_iterator ci=delaunay.all_cells_begin(), ce=delaunay.all_cells_end(); ci!=ce; ++ci) {
			const cell_size_t ciID(ci->info());
			const cell_info_t& ciInfo(infoCells[ciID]);
			graph.AddNode(ciID, ciInfo.s, MINF(ciInfo.t, maxCap));
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
		// scale-aware webbing gate: every Delaunay vertex IS an input point, so a facet can
		// stray far from the observed cloud only by spanning it with long edges - the surface
		// a visibility mesh grows across occluded space (under vehicles, behind walls) that no
		// observation supports; measure each cut facet by its longest squared edge and drop
		// the ones beyond maxEdgeScale x the median cut-facet longest edge (both live in the
		// working space so canonical rescale cancels, and the ratio is scene-independent)
		const auto maxFacetEdgeSq([&delaunay](const cell_handle_t& ci, int i) {
			const auto tri(delaunay.triangle(ci, i));
			return (float)MAXF3(CGAL::squared_distance(tri[0], tri[1]),
			                    CGAL::squared_distance(tri[1], tri[2]),
			                    CGAL::squared_distance(tri[2], tri[0]));
		});
		float gateEdgeSq(FLT_MAX);
		if (params.maxEdgeScale > 0) {
			FloatArr edgesSq(0, nEstimatedNumVerts*2);
			for (delaunay_t::All_cells_iterator ci=delaunay.all_cells_begin(), ce=delaunay.all_cells_end(); ci!=ce; ++ci) {
				const cell_size_t ciID(ci->info());
				for (int i=0; i<4; ++i) {
					if (delaunay.is_infinite(ci, i)) continue;
					const cell_handle_t cj(ci->neighbor(i));
					const cell_size_t cjID(cj->info());
					if (ciID < cjID) continue;
					if (graph.IsNodeOnSrcSide(ciID) == graph.IsNodeOnSrcSide(cjID)) continue;
					edgesSq.Insert(maxFacetEdgeSq(ci, i));
				}
			}
			if (!edgesSq.IsEmpty())
				gateEdgeSq = edgesSq.GetMedian()*SQUARE(params.maxEdgeScale);
		}
		size_t numUnsupportedFaces(0);
		for (delaunay_t::All_cells_iterator ci=delaunay.all_cells_begin(), ce=delaunay.all_cells_end(); ci!=ce; ++ci) {
			const cell_size_t ciID(ci->info());
			for (int i=0; i<4; ++i) {
				if (delaunay.is_infinite(ci, i)) continue;
				const cell_handle_t cj(ci->neighbor(i));
				const cell_size_t cjID(cj->info());
				if (ciID < cjID) continue;
				const bool ciType(graph.IsNodeOnSrcSide(ciID));
				if (ciType == graph.IsNodeOnSrcSide(cjID)) continue;
				if (params.maxEdgeScale > 0 && maxFacetEdgeSq(ci, i) > gateEdgeSq) {
					++numUnsupportedFaces;
					continue;
				}
				Mesh::Face& face = mesh.faces.AddEmpty();
				const triangle_vhandles_t tri(getTriangle(ci, i));
				for (int v=0; v<3; ++v) {
					const vertex_handle_t vh(tri.verts[v]);
					ASSERT(vh->point() == delaunay.triangle(ci,i)[v]);
					const auto pairItID(mapVertices.insert(std::make_pair(vh.for_compact_container(), (Mesh::VIndex)mesh.vertices.GetSize())));
					if (pairItID.second)
						mesh.vertices.Insert(CGAL2MVS<Mesh::Vertex::Type>(rescale.ToWorld(vh->point())));
					ASSERT(pairItID.first->second < mesh.vertices.GetSize());
					face[v] = pairItID.first->second;
				}
				// correct face orientation
				if (!ciType)
					std::swap(face[0], face[2]);
			}
		}
		delaunay.clear();
		if (params.maxEdgeScale > 0)
			DEBUG_EXTRA("Unsupported surface facets removed: %u (longest edge > %g x median)", (unsigned)numUnsupportedFaces, params.maxEdgeScale);

		DEBUG_EXTRA("Delaunay tetrahedras graph-cut completed (%g flow): %u vertices, %u faces (%s)", maxflow, mesh.vertices.GetSize(), mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());
	}

	// fix non-manifold vertices and edges;
	// a single pass is exhaustive: each vertex is split into one duplicate per incident
	// connected component of faces (itself manifold by construction), and splitting never
	// alters the incident-face set of any other vertex, so no vertex needs to be revisited
	mesh.FixNonManifold();
	return true;
}
/*----------------------------------------------------------------*/

#pragma pop_macro("VERBOSE")
