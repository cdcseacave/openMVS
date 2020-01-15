/*
* Scene.cpp
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
#define _USE_OPENCV
#include "Interface.h"

using namespace MVS;


// D E F I N E S ///////////////////////////////////////////////////

#define PROJECT_ID "MVS\0" // identifies the project stream
#define PROJECT_VER ((uint32_t)1) // identifies the version of a project stream


// S T R U C T S ///////////////////////////////////////////////////

void Scene::Release()
{
	platforms.Release();
	images.Release();
	pointcloud.Release();
	mesh.Release();
}

bool Scene::IsEmpty() const
{
	return pointcloud.IsEmpty() && mesh.IsEmpty();
}


bool Scene::LoadInterface(const String & fileName)
{
	TD_TIMER_STARTD();
	Interface obj;

	// serialize in the current state
	if (!ARCHIVE::SerializeLoad(obj, fileName))
		return false;

	// import platforms and cameras
	ASSERT(!obj.platforms.empty());
	platforms.Reserve((uint32_t)obj.platforms.size());
	for (const Interface::Platform& itPlatform: obj.platforms) {
		Platform& platform = platforms.AddEmpty();
		platform.name = itPlatform.name;
		platform.cameras.Reserve((uint32_t)itPlatform.cameras.size());
		for (const Interface::Platform::Camera& itCamera: itPlatform.cameras) {
			Platform::Camera& camera = platform.cameras.AddEmpty();
			camera.K = itCamera.K;
			camera.R = itCamera.R;
			camera.C = itCamera.C;
			if (!itCamera.IsNormalized()) {
				// normalize K
				ASSERT(itCamera.HasResolution());
				const REAL scale(REAL(1)/camera.GetNormalizationScale(itCamera.width,itCamera.height));
				camera.K(0,0) *= scale;
				camera.K(1,1) *= scale;
				camera.K(0,2) *= scale;
				camera.K(1,2) *= scale;
			}
			DEBUG_EXTRA("Camera model loaded: platform %u; camera %2u; f %.3fx%.3f; poses %u", platforms.size()-1, platform.cameras.size()-1, camera.K(0,0), camera.K(1,1), itPlatform.poses.size());
		}
		ASSERT(platform.cameras.GetSize() == itPlatform.cameras.size());
		platform.poses.Reserve((uint32_t)itPlatform.poses.size());
		for (const Interface::Platform::Pose& itPose: itPlatform.poses) {
			Platform::Pose& pose = platform.poses.AddEmpty();
			pose.R = itPose.R;
			pose.C = itPose.C;
		}
		ASSERT(platform.poses.GetSize() == itPlatform.poses.size());
	}
	ASSERT(platforms.GetSize() == obj.platforms.size());
	if (platforms.IsEmpty())
		return false;

	// import images
	nCalibratedImages = 0;
	size_t nTotalPixels(0);
	ASSERT(!obj.images.empty());
	images.Reserve((uint32_t)obj.images.size());
	for (const Interface::Image& image: obj.images) {
		const uint32_t ID(images.size());
		Image& imageData = images.AddEmpty();
		imageData.name = image.name;
		Util::ensureUnifySlash(imageData.name);
		imageData.name = MAKE_PATH_FULL(WORKING_FOLDER_FULL, imageData.name);
		imageData.poseID = image.poseID;
		if (imageData.poseID == NO_ID) {
			DEBUG_EXTRA("warning: uncalibrated image '%s'", image.name.c_str());
			continue;
		}
		imageData.platformID = image.platformID;
		imageData.cameraID = image.cameraID;
		imageData.ID = (image.ID == NO_ID ? ID : image.ID);
		// init camera
		const Interface::Platform::Camera& camera = obj.platforms[image.platformID].cameras[image.cameraID];
		if (camera.HasResolution()) {
			// use stored resolution
			imageData.width = camera.width;
			imageData.height = camera.height;
			imageData.scale = 1;
		} else {
			// read image header for resolution
			if (!imageData.ReloadImage(0, false))
				return false;
		}
		imageData.UpdateCamera(platforms);
		++nCalibratedImages;
		nTotalPixels += imageData.width * imageData.height;
		DEBUG_EXTRA("Image loaded %3u: %s", ID, Util::getFileNameExt(imageData.name).c_str());
	}
	if (images.GetSize() < 2)
		return false;

	// import 3D points
	if (!obj.vertices.empty()) {
		bool bValidWeights(false);
		pointcloud.points.Resize(obj.vertices.size());
		pointcloud.pointViews.Resize(obj.vertices.size());
		pointcloud.pointWeights.Resize(obj.vertices.size());
		FOREACH(i, pointcloud.points) {
			const Interface::Vertex& vertex = obj.vertices[i];
			PointCloud::Point& point = pointcloud.points[i];
			point = vertex.X;
			PointCloud::ViewArr& views = pointcloud.pointViews[i];
			views.Resize((PointCloud::ViewArr::IDX)vertex.views.size());
			PointCloud::WeightArr& weights = pointcloud.pointWeights[i];
			weights.Resize((PointCloud::ViewArr::IDX)vertex.views.size());
			CLISTDEF0(PointCloud::ViewArr::IDX) indices(views.GetSize());
			std::iota(indices.Begin(), indices.End(), 0);
			std::sort(indices.Begin(), indices.End(), [&](IndexArr::Type i0, IndexArr::Type i1) -> bool {
				return vertex.views[i0].imageID < vertex.views[i1].imageID;
			});
			ASSERT(vertex.views.size() >= 2);
			views.ForEach([&](PointCloud::ViewArr::IDX v) {
				const Interface::Vertex::View& view = vertex.views[indices[v]];
				views[v] = view.imageID;
				weights[v] = view.confidence;
				if (view.confidence != 0)
					bValidWeights = true;
			});
		}
		if (!bValidWeights)
			pointcloud.pointWeights.Release();
		if (!obj.verticesNormal.empty()) {
			ASSERT(obj.vertices.size() == obj.verticesNormal.size());
			pointcloud.normals.CopyOf((const Point3f*)&obj.verticesNormal[0].n, obj.vertices.size());
		}
		if (!obj.verticesColor.empty()) {
			ASSERT(obj.vertices.size() == obj.verticesColor.size());
			pointcloud.colors.CopyOf((const Pixel8U*)&obj.verticesColor[0].c, obj.vertices.size());
		}
	}

	DEBUG_EXTRA("Scene loaded from interface format (%s):\n"
				"\t%u images (%u calibrated) with a total of %.2f MPixels (%.2f MPixels/image)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.GetSize(), nCalibratedImages, (double)nTotalPixels/(1024.0*1024.0), (double)nTotalPixels/(1024.0*1024.0*nCalibratedImages),
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
	return true;
} // LoadInterface

bool Scene::SaveInterface(const String & fileName, int version) const
{
	TD_TIMER_STARTD();
	Interface obj;

	// export platforms
	obj.platforms.reserve(platforms.GetSize());
	for (const Platform& platform: platforms) {
		Interface::Platform plat;
		plat.cameras.reserve(platform.cameras.GetSize());
		for (const Platform::Camera& camera: platform.cameras) {
			Interface::Platform::Camera cam;
			cam.K = camera.K;
			cam.R = camera.R;
			cam.C = camera.C;
			plat.cameras.push_back(cam);
		}
		plat.poses.reserve(platform.poses.GetSize());
		for (const Platform::Pose& pose: platform.poses) {
			Interface::Platform::Pose p;
			p.R = pose.R;
			p.C = pose.C;
			plat.poses.push_back(p);
		}
		obj.platforms.push_back(plat);
	}

	// export images
	obj.images.resize(images.GetSize());
	FOREACH(i, images) {
		const Image& imageData = images[i];
		MVS::Interface::Image& image = obj.images[i];
		image.name = MAKE_PATH_REL(WORKING_FOLDER_FULL, imageData.name);
		image.poseID = imageData.poseID;
		image.platformID = imageData.platformID;
		image.cameraID = imageData.cameraID;
		image.ID = imageData.ID;
	}

	// export 3D points
	obj.vertices.resize(pointcloud.points.GetSize());
	FOREACH(i, pointcloud.points) {
		const PointCloud::Point& point = pointcloud.points[i];
		const PointCloud::ViewArr& views = pointcloud.pointViews[i];
		MVS::Interface::Vertex& vertex = obj.vertices[i];
		ASSERT(sizeof(vertex.X.x) == sizeof(point.x));
		vertex.X = point;
		vertex.views.resize(views.GetSize());
		views.ForEach([&](PointCloud::ViewArr::IDX v) {
			MVS::Interface::Vertex::View& view = vertex.views[v];
			view.imageID = views[v];
			view.confidence = (pointcloud.pointWeights.IsEmpty() ? 0.f : pointcloud.pointWeights[i][v]);
		});
	}
	if (!pointcloud.normals.IsEmpty()) {
		obj.verticesNormal.resize(pointcloud.normals.GetSize());
		FOREACH(i, pointcloud.normals) {
			const PointCloud::Normal& normal = pointcloud.normals[i];
			MVS::Interface::Normal& vertexNormal = obj.verticesNormal[i];
			vertexNormal.n = normal;
		}
	}
	if (!pointcloud.colors.IsEmpty()) {
		obj.verticesColor.resize(pointcloud.colors.GetSize());
		FOREACH(i, pointcloud.colors) {
			const PointCloud::Color& color = pointcloud.colors[i];
			MVS::Interface::Color& vertexColor = obj.verticesColor[i];
			vertexColor.c = color;
		}
	}

	// serialize out the current state
	if (!ARCHIVE::SerializeSave(obj, fileName, version>=0?uint32_t(version):MVSI_PROJECT_VER))
		return false;

	DEBUG_EXTRA("Scene saved to interface format (%s):\n"
				"\t%u images (%u calibrated)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.GetSize(), nCalibratedImages,
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
	return true;
} // SaveInterface
/*----------------------------------------------------------------*/

// try to load known point-cloud or mesh files
bool Scene::Import(const String& fileName)
{
	const String ext(Util::getFileExt(fileName).ToLower());
	if (ext == _T(".obj")) {
		// import mesh from obj file
		Release();
		return mesh.Load(fileName);
	}
	if (ext == _T(".ply")) {
		// import point-cloud/mesh from ply file
		Release();
		int nVertices(0), nFaces(0);
		{
		PLY ply;
		if (!ply.read(fileName)) {
			DEBUG_EXTRA("error: invalid PLY file");
			return false;
		}
		for (int i = 0; i < (int)ply.elems.size(); ++i) {
			int elem_count;
			LPCSTR elem_name = ply.setup_element_read(i, &elem_count);
			if (PLY::equal_strings("vertex", elem_name)) {
				nVertices = elem_count;
			} else
			if (PLY::equal_strings("face", elem_name)) {
				nFaces = elem_count;
			}
		}
		}
		if (nVertices && nFaces)
			return mesh.Load(fileName);
		if (nVertices)
			return pointcloud.Load(fileName);
	}
	return false;
} // Import
/*----------------------------------------------------------------*/

bool Scene::Load(const String& fileName, bool bImport)
{
	TD_TIMER_STARTD();
	Release();

	#ifdef _USE_BOOST
	// open the input stream
	std::ifstream fs(fileName, std::ios::in | std::ios::binary);
	if (!fs.is_open())
		return false;
	// load project header ID
	char szHeader[4];
	fs.read(szHeader, 4);
	if (!fs || _tcsncmp(szHeader, PROJECT_ID, 4) != 0) {
		fs.close();
		if (bImport && Import(fileName))
			return true;
		if (LoadInterface(fileName))
			return true;
		VERBOSE("error: invalid project");
		return false;
	}
	// load project version
	uint32_t nVer;
	fs.read((char*)&nVer, sizeof(uint32_t));
	if (!fs || nVer != PROJECT_VER) {
		VERBOSE("error: different project version");
		return false;
	}
	// load stream type
	uint32_t nType;
	fs.read((char*)&nType, sizeof(uint32_t));
	// skip reserved bytes
	uint64_t nReserved;
	fs.read((char*)&nReserved, sizeof(uint64_t));
	// serialize in the current state
	if (!SerializeLoad(*this, fs, (ARCHIVE_TYPE)nType))
		return false;
	// init images
	nCalibratedImages = 0;
	size_t nTotalPixels(0);
	FOREACH(ID, images) {
		Image& imageData = images[ID];
		if (imageData.poseID == NO_ID)
			continue;
		imageData.UpdateCamera(platforms);
		++nCalibratedImages;
		nTotalPixels += imageData.width * imageData.height;
	}
	DEBUG_EXTRA("Scene loaded (%s):\n"
				"\t%u images (%u calibrated) with a total of %.2f MPixels (%.2f MPixels/image)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.GetSize(), nCalibratedImages, (double)nTotalPixels/(1024.0*1024.0), (double)nTotalPixels/(1024.0*1024.0*nCalibratedImages),
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
	return true;
	#else
	return false;
	#endif
} // Load

bool Scene::Save(const String& fileName, ARCHIVE_TYPE type) const
{
	TD_TIMER_STARTD();
	// save using MVS interface if requested
	if (type == ARCHIVE_MVS) {
		if (mesh.IsEmpty())
			return SaveInterface(fileName);
		type = ARCHIVE_BINARY_ZIP;
	}
	#ifdef _USE_BOOST
	// open the output stream
	std::ofstream fs(fileName, std::ios::out | std::ios::binary);
	if (!fs.is_open())
		return false;
	// save project ID
	fs.write(PROJECT_ID, 4);
	// save project version
	const uint32_t nVer(PROJECT_VER);
	fs.write((const char*)&nVer, sizeof(uint32_t));
	// save stream type
	const uint32_t nType = type;
	fs.write((const char*)&nType, sizeof(uint32_t));
	// reserve some bytes
	const uint64_t nReserved = 0;
	fs.write((const char*)&nReserved, sizeof(uint64_t));
	// serialize out the current state
	if (!SerializeSave(*this, fs, type))
		return false;
	DEBUG_EXTRA("Scene saved (%s):\n"
				"\t%u images (%u calibrated)\n"
				"\t%u points, %u vertices, %u faces",
				TD_TIMER_GET_FMT().c_str(),
				images.GetSize(), nCalibratedImages,
				pointcloud.points.GetSize(), mesh.vertices.GetSize(), mesh.faces.GetSize());
	return true;
	#else
	return false;
	#endif
} // Save
/*----------------------------------------------------------------*/


inline float Footprint(const Camera& camera, const Point3f& X) {
	#if 0
	const REAL fSphereRadius(1);
	const Point3 cX(camera.TransformPointW2C(Cast<REAL>(X)));
	return (float)norm(camera.TransformPointC2I(Point3(cX.x+fSphereRadius,cX.y,cX.z))-camera.TransformPointC2I(cX))+std::numeric_limits<float>::epsilon();
	#else
	return (float)(camera.GetFocalLength()/camera.PointDepth(X));
	#endif
}

// compute visibility for the reference image
// and select the best views for reconstructing the dense point-cloud;
// extract also all 3D points seen by the reference image;
// (inspired by: "Multi-View Stereo for Community Photo Collections", Goesele, 2007)
bool Scene::SelectNeighborViews(uint32_t ID, IndexArr& points, unsigned nMinViews, unsigned nMinPointViews, float fOptimAngle)
{
	ASSERT(points.IsEmpty());

	// extract the estimated 3D points and the corresponding 2D projections for the reference image
	Image& imageData = images[ID];
	ASSERT(imageData.IsValid());
	ViewScoreArr& neighbors = imageData.neighbors;
	ASSERT(neighbors.IsEmpty());
	struct Score {
		float score;
		float avgScale;
		float avgAngle;
		uint32_t points;
	};
	CLISTDEF0(Score) scores(images.GetSize());
	scores.Memset(0);
	if (nMinPointViews > nCalibratedImages)
		nMinPointViews = nCalibratedImages;
	unsigned nPoints = 0;
	imageData.avgDepth = 0;
	FOREACH(idx, pointcloud.points) {
		const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
		ASSERT(views.IsSorted());
		if (views.FindFirst(ID) == PointCloud::ViewArr::NO_INDEX)
			continue;
		// store this point
		const PointCloud::Point& point = pointcloud.points[idx];
		if (views.GetSize() >= nMinPointViews)
			points.Insert((uint32_t)idx);
		imageData.avgDepth += (float)imageData.camera.PointDepth(point);
		++nPoints;
		// score shared views
		const Point3f V1(imageData.camera.C - Cast<REAL>(point));
		const float footprint1(Footprint(imageData.camera, point));
		for (const PointCloud::View& view: views) {
			if (view == ID)
				continue;
			const Image& imageData2 = images[view];
			const Point3f V2(imageData2.camera.C - Cast<REAL>(point));
			const float fAngle(ACOS(ComputeAngle<float,float>(V1.ptr(), V2.ptr())));
			const float wAngle(MINF(POW(fAngle/fOptimAngle, 1.5f), 1.f));
			const float footprint2(Footprint(imageData2.camera, point));
			const float fScaleRatio(footprint1/footprint2);
			float wScale;
			if (fScaleRatio > 1.6f)
				wScale = SQUARE(1.6f/fScaleRatio);
			else if (fScaleRatio >= 1.f)
				wScale = 1.f;
			else
				wScale = SQUARE(fScaleRatio);
			Score& score = scores[view];
			score.score += wAngle * wScale;
			score.avgScale += fScaleRatio;
			score.avgAngle += fAngle;
			++score.points;
		}
	}
	imageData.avgDepth /= nPoints;
	ASSERT(nPoints > 3);

	// select best neighborViews
	Point2fArr projs(0, points.GetSize());
	FOREACH(IDB, images) {
		const Image& imageDataB = images[IDB];
		if (!imageDataB.IsValid())
			continue;
		const Score& score = scores[IDB];
		if (score.points < 3)
			continue;
		ASSERT(ID != IDB);
		// compute how well the matched features are spread out (image covered area)
		const Point2f boundsA(imageData.GetSize());
		const Point2f boundsB(imageDataB.GetSize());
		ASSERT(projs.IsEmpty());
		for (uint32_t idx: points) {
			const PointCloud::ViewArr& views = pointcloud.pointViews[idx];
			ASSERT(views.IsSorted());
			ASSERT(views.FindFirst(ID) != PointCloud::ViewArr::NO_INDEX);
			if (views.FindFirst(IDB) == PointCloud::ViewArr::NO_INDEX)
				continue;
			const PointCloud::Point& point = pointcloud.points[idx];
			Point2f& ptA = projs.AddConstruct(imageData.camera.ProjectPointP(point));
			Point2f ptB = imageDataB.camera.ProjectPointP(point);
			if (!imageData.camera.IsInside(ptA, boundsA) || !imageDataB.camera.IsInside(ptB, boundsB))
				projs.RemoveLast();
		}
		ASSERT(projs.GetSize() <= score.points);
		if (projs.IsEmpty())
			continue;
		const float area(ComputeCoveredArea<float,2,16,false>((const float*)projs.Begin(), projs.GetSize(), boundsA.ptr()));
		projs.Empty();
		// store image score
		ViewScore& neighbor = neighbors.AddEmpty();
		neighbor.idx.ID = IDB;
		neighbor.idx.points = score.points;
		neighbor.idx.scale = score.avgScale/score.points;
		neighbor.idx.angle = score.avgAngle/score.points;
		neighbor.idx.area = area;
		neighbor.score = score.score*area;
	}
	neighbors.Sort();
	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print neighbor views
	if (VERBOSITY_LEVEL > 2) {
		String msg;
		FOREACH(n, neighbors)
			msg += String::FormatString(" %3u(%upts,%.2fscl)", neighbors[n].idx.ID, neighbors[n].idx.points, neighbors[n].idx.scale);
		VERBOSE("Reference image %3u sees %u views:%s (%u shared points)", ID, neighbors.GetSize(), msg.c_str(), nPoints);
	}
	#endif
	if (points.GetSize() <= 3 || neighbors.GetSize() < MINF(nMinViews,nCalibratedImages-1)) {
		DEBUG_EXTRA("error: reference image %3u has not enough images in view", ID);
		return false;
	}
	return true;
} // SelectNeighborViews
/*----------------------------------------------------------------*/

// keep only the best neighbors for the reference image
bool Scene::FilterNeighborViews(ViewScoreArr& neighbors, float fMinArea, float fMinScale, float fMaxScale, float fMinAngle, float fMaxAngle, unsigned nMaxViews)
{
	// remove invalid neighbor views
	RFOREACH(n, neighbors) {
		const ViewScore& neighbor = neighbors[n];
		if (neighbor.idx.area < fMinArea ||
			!ISINSIDE(neighbor.idx.scale, fMinScale, fMaxScale) ||
			!ISINSIDE(neighbor.idx.angle, fMinAngle, fMaxAngle))
			neighbors.RemoveAtMove(n);
	}
	if (neighbors.GetSize() > nMaxViews)
		neighbors.Resize(nMaxViews);
	return !neighbors.IsEmpty();
} // FilterNeighborViews
/*----------------------------------------------------------------*/


// export all estimated cameras in a MeshLab MLP project as raster layers
bool Scene::ExportCamerasMLP(const String& fileName, const String& fileNameScene) const
{
	static const char mlp_header[] =
		"<!DOCTYPE MeshLabDocument>\n"
		"<MeshLabProject>\n"
		" <MeshGroup>\n"
		"  <MLMesh label=\"%s\" filename=\"%s\">\n"
		"   <MLMatrix44>\n"
		"1 0 0 0 \n"
		"0 1 0 0 \n"
		"0 0 1 0 \n"
		"0 0 0 1 \n"
		"   </MLMatrix44>\n"
		"  </MLMesh>\n"
		" </MeshGroup>\n";
	static const char mlp_raster_pos[] =
		"  <MLRaster label=\"%s\">\n"
		"   <VCGCamera TranslationVector=\"%0.6g %0.6g %0.6g 1\"";
	static const char mlp_raster_cam[] =
		" LensDistortion=\"%0.6g %0.6g\""
		" ViewportPx=\"%u %u\""
		" PixelSizeMm=\"1 %0.4f\""
		" FocalMm=\"%0.4f\""
		" CenterPx=\"%0.4f %0.4f\"";
	static const char mlp_raster_rot[] =
		" RotationMatrix=\"%0.6g %0.6g %0.6g 0 %0.6g %0.6g %0.6g 0 %0.6g %0.6g %0.6g 0 0 0 0 1\"/>\n"
		"   <Plane semantic=\"\" fileName=\"%s\"/>\n"
		"  </MLRaster>\n";

	Util::ensureFolder(fileName);
	File f(fileName, File::WRITE, File::CREATE | File::TRUNCATE);

	// write MLP header containing the referenced PLY file
	f.print(mlp_header, Util::getFileName(fileNameScene).c_str(), MAKE_PATH_REL(WORKING_FOLDER_FULL, fileNameScene).c_str());

	// write the raster layers
	f <<  " <RasterGroup>\n";
	FOREACH(i, images) {
		const Image& imageData = images[i];
		// skip invalid, uncalibrated or discarded images
		if (!imageData.IsValid())
			continue;
		const Camera& camera = imageData.camera;
		f.print(mlp_raster_pos,
			Util::getFileName(imageData.name).c_str(),
			-camera.C.x, -camera.C.y, -camera.C.z
		);
		f.print(mlp_raster_cam,
			0, 0,
			imageData.width, imageData.height,
			camera.K(1,1)/camera.K(0,0), camera.K(0,0),
			camera.K(0,2), camera.K(1,2)
		);
		f.print(mlp_raster_rot,
			 camera.R(0,0),  camera.R(0,1),  camera.R(0,2),
			-camera.R(1,0), -camera.R(1,1), -camera.R(1,2),
			-camera.R(2,0), -camera.R(2,1), -camera.R(2,2),
			MAKE_PATH_REL(WORKING_FOLDER_FULL, imageData.name).c_str()
		);
	}
	f << " </RasterGroup>\n</MeshLabProject>\n";

	return true;
} // ExportCamerasMLP
/*----------------------------------------------------------------*/
