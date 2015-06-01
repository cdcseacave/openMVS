/*
* SceneReconstruct.cpp
*
* Copyright (c) 2014-2015 FOXEL SA - http://foxel.ch
* Please read <http://foxel.ch/license> for more information.
*
*
* Author(s):
*
*      cDc <cdc.seacave@gmail.com>
*
*
* This file is part of the FOXEL project <http://foxel.ch>.
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
*
*      You are required to attribute the work as explained in the "Usage and
*      Attribution" section of <http://foxel.ch/license>.
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

#define WEIGHT_INF  (uint32_t)(INT_MAX/8)


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
	typedef SEACAVE::cList<uint32_t,uint32_t,0,4,uint32_t> view_vec_t;
	view_vec_t views; // faces' weight from the cell outwards
	Type w; // point's weight
	inline vert_info_t() : w(0) {}
	void Insert(uint32_t viewID) {
		// insert viewID in increasing order
		const uint32_t idx((uint32_t)views.FindFirstEqlGreater(viewID));
		if (idx < views.GetSize() && views[idx] == viewID) {
			// the new view is already in the array
			ASSERT(views.FindFirst(viewID) == idx);
		} else {
			// the new view is not in the array,
			// insert it
			views.InsertAt(idx, viewID);
			ASSERT(views.IsSorted());
		}
		// update point's weight
		w += 1;
	}
};

struct cell_info_t {
	typedef edge_cap_t Type;
	Type f[4]; // faces' weight from the cell outwards
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

struct camera_cell_t {
	cell_handle_t cell; // cell containing the camera
	std::vector<facet_t> facets; // all facets on the convex-hull in view of the camera (ordered by importance)
};

struct adjacent_vertex_back_inserter_t {
	const delaunay_t& delaunay;
	const point_t& p;
	vertex_handle_t& v;
	inline adjacent_vertex_back_inserter_t(const delaunay_t& _delaunay, const point_t& _p, vertex_handle_t& _v) : delaunay(_delaunay), p(_p), v(_v) {}
	inline adjacent_vertex_back_inserter_t& operator*() { return (*this); }
	inline adjacent_vertex_back_inserter_t operator++(int) { return *this; }
	inline void operator=(const vertex_handle_t& w) {
		if (delaunay.is_infinite(v) || delaunay.geom_traits().compare_distance_3_object()(p, w->point(), v->point()) == CGAL::SMALLER)
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


// Check if a point (p) is coplanar with a triangle (a, b, c);
// return orientation type
inline int orientation(const point_t& a, const point_t& b, const point_t& c, const point_t& p)
{
	#if 0
	return CGAL::orientation(a, b, c, p);
	#else
	// inexact_orientation
	const double& px = a.x(); const double& py = a.y(); const double& pz = a.z();
	const double& qx = b.x(); const double& qy = b.y(); const double& qz = b.z();
	const double& rx = c.x(); const double& ry = c.y(); const double& rz = c.z();
	const double& sx = p.x(); const double& sy = p.y(); const double& sz = p.z();
	const double det = CGAL::determinant(
		qx - px, qy - py, qz - pz,
		rx - px, ry - py, rz - pz,
		sx - px, sy - py, sz - pz);
	#if 0
	if (det > ZERO_TOLERANCE) return CGAL::POSITIVE;
	if (det < ZERO_TOLERANCE) return CGAL::NEGATIVE;
	return CGAL::orientation(a, b, c, p);
	#else
	if (det > 0) return CGAL::POSITIVE;
	if (det < 0) return CGAL::NEGATIVE;
	return CGAL::COPLANAR;
	#endif
	#endif
}

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

// find which facet is intersected by the segment (seg)
bool intersect(const delaunay_t& Tr, const segment_t& seg, const std::vector<facet_t>& facets)
{
	ASSERT(!facets.empty());
	int coplanar[3];
	for (std::vector<facet_t>::const_iterator it=facets.cbegin(); it!=facets.cend(); ++it) {
		ASSERT(!Tr.is_infinite(*it));
		// i
		facet_t facet;
		if (coplanar[0]) {
			// face intersection
			const cell_handle_t nc(facet.first->neighbor(facet.second));
			ASSERT(!Tr.is_infinite(nc));
			for (int i=0; i<4; ++i)
				if (nc->neighbor(i) != facet.first)
					//facets.push_back(facet_t(nc, i));
			return true;
		}
	}
	//facets.clear();
	return false;
}
} // namespace DELAUNAY

// First, iteratively create a Delaunay triangulation of the existing point-cloud by inserting point by point,
// iif the point to be inserted is not closer than distInsert pixels in at least one of its views to
// the projection of any of already inserted points.
// Next, the score is computed for all the edges of the directed graph composed of points as vertices.
// Finally, graph-cut algorithm is used to split the tetrahedrons in inside and outside,
// and the surface is such extracted.
bool Scene::ReconstructMesh(float distInsert)
{
	using namespace DELAUNAY;
	ASSERT(!pointcloud.IsEmpty());

	// create the Delaunay triangulation
	delaunay_t delaunay;
	std::vector<cell_info_t> infoCells;
	std::vector<camera_cell_t> camCells;
	{
		TD_TIMER_STARTD();

		std::vector<point_t> vertices(pointcloud.points.GetSize());
		std::vector<std::ptrdiff_t> indices(pointcloud.points.GetSize());
		// fetch points
		FOREACH(i, pointcloud.points) {
			const Point3f& X(pointcloud.points[i].X);
			vertices[i] = point_t(X.x, X.y, X.z);
			indices[i] = i;
		}
		// sort vertices
		typedef CGAL::Spatial_sort_traits_adapter_3<delaunay_t::Geom_traits, point_t*> Search_traits;
		CGAL::spatial_sort(indices.begin(), indices.end(), Search_traits(&vertices[0], delaunay.geom_traits()));
		// insert vertices
		const float distInsertSq(SQUARE(distInsert));
		vertex_handle_t hint;
		delaunay_t::Locate_type lt;
		int li, lj;
		std::for_each(indices.cbegin(), indices.cend(), [&](size_t idx) {
			const point_t& p = vertices[idx];
			const PointCloud::Point& point = pointcloud.points[idx];
			ASSERT(!point.views.IsEmpty());
			if (hint == vertex_handle_t()) {
				// this is the first point,
				// insert it
				hint = delaunay.insert(p);
				ASSERT(hint != vertex_handle_t());
			} else {
				// locate cell containing this point
				cell_handle_t c(delaunay.locate(p, lt, li, lj, hint->cell()));
				if (lt == delaunay_t::VERTEX) {
					// duplicate point, nothing to insert,
					// just update its visibility info
					hint = c->vertex(li);
					ASSERT(hint != delaunay.infinite_vertex());
				} else {
					if (distInsert <= 0) {
						// insert all points
						hint = delaunay.insert(p, hint);
						ASSERT(hint != vertex_handle_t());
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
								vertex_handle_t v(nearest);
								delaunay.adjacent_vertices(nearest, adjacent_vertex_back_inserter_t(delaunay, p, v));
								if (v == nearest)
									break;
								nearest = v;
							}
						}
						ASSERT(nearest == delaunay.nearest_vertex(p));
						// check if point is far enough to all existing points
						FOREACHPTR(pViewID, point.views) {
							const Image& imageData = images[*pViewID];
							const Point2f pn(imageData.camera.ProjectPointP(point.X));
							const Point2f pe(imageData.camera.ProjectPointP(CGAL2MVS<float>(nearest->point())));
							if (normSq(pn-pe) > distInsertSq) {
								// point far enough to an existing point,
								// insert as a new point
								hint = delaunay.insert(p, lt, c, li, lj);
								ASSERT(hint != vertex_handle_t());
								break;
							}
						}
					}
				}
			}
			// update point visibility info
			vert_info_t& vi = hint->info();
			FOREACHPTR(pImageID, point.views) {
				ASSERT(images[*pImageID].IsValid());
				ASSERT(images[*pImageID].camera.IsInsideProjectionP(CGAL2MVS<REAL>(p)));
				vi.Insert(*pImageID);
			}
		});
		pointcloud.Release();
		// init cells weights
		const size_t numNodes(delaunay.number_of_cells());
		infoCells.resize(numNodes);
		memset(&infoCells[0], 0, sizeof(cell_info_t)*numNodes);
		cell_size_t ciID(0);
		for (delaunay_t::All_cells_iterator ci=delaunay.all_cells_begin(), eci=delaunay.all_cells_end(); ci!=eci; ++ci, ++ciID)
			ci->info() = ciID;

		DEBUG_EXTRA("Delaunay tetrahedralization completed: %u points -> %u vertices, %u (+%u) cells, %u (+%u) faces (%s)", indices.size(), delaunay.number_of_vertices(), delaunay.number_of_finite_cells(), delaunay.number_of_cells()-delaunay.number_of_finite_cells(), delaunay.number_of_finite_facets(), delaunay.number_of_facets()-delaunay.number_of_finite_facets(), TD_TIMER_GET_FMT().c_str());
	}


	// for every camera-point ray intersect it with the tetrahedrons and
	// add alpha_vis(point) to cell's directed edge in the graph
	{
		TD_TIMER_STARTD();

		// estimate the size of the smallest reconstructible object
		FloatArr dists(0, delaunay.number_of_edges());
		for (delaunay_t::Finite_edges_iterator ei=delaunay.finite_edges_begin(), eei=delaunay.finite_edges_end(); ei!=eei; ++ei) {
			const cell_handle_t& c(ei->first);
			dists.Insert(normSq(CGAL2MVS<float>(c->vertex(ei->second)->point()) - CGAL2MVS<float>(c->vertex(ei->third)->point())));
		}
		const float sigma(dists.GetNth(dists.GetSize()/2)*2);
		const float inv2SigmaSq(0.5f/(sigma*sigma));
		dists.Release();


		// compute the weights for each edge
		std::vector<facet_t> facets;
		for (delaunay_t::Vertex_iterator vi=delaunay.vertices_begin(), vie=delaunay.vertices_end(); vi!=vie; ++vi) {
			vert_info_t& vert(vi->info());
			if (vert.views.IsEmpty())
				continue;
			const edge_cap_t alpha_vis(vert.w);
			const point_t& p(vi->point());
			const Point3f pt(CGAL2MVS<float>(p));
			FOREACH(v, vert.views) {
				const uint32_t imageID(vert.views[(vert_info_t::view_vec_t::IDX)v]);
				const Image& imageData = images[imageID];
				ASSERT(imageData.IsValid());
				const Camera& camera = imageData.camera;
				const camera_cell_t& camCell = camCells[imageID];
				// compute the ray used to find point intersection
				const Point3f vecCamPoint(pt-CastFloat(camera.C));
				const float invLenCamPoint(1.f/norm(vecCamPoint));
				// find faces intersected by the camera-point segment
				const segment_t segCamPoint(MVS2CGAL(camera.C), p);
				if (!intersect(delaunay, segCamPoint, camCell.facets))
					continue;
			}
		}
	}

	// run graph-cut and extract the mesh
	{
	}

	return true;
}
/*----------------------------------------------------------------*/
