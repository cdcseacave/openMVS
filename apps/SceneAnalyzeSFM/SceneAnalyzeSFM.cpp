/*
 * SceneAnalyzeSFM.cpp
 *
 * Copyright (c) 2014-2026 SEACAVE
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

// Campaign instrumentation (roma2-matching-redesign, task 6a): a read-only project dump used to
// measure the RoMa2 dense-supplementation campaign offline. It loads a saved SFM project and emits
// one CSV per entity (tracks, observations, pairs, images) beside it, plus matches.bin, every stored
// match as pixel coordinates -- no pipeline change, nothing is written back into the project. Deliberately self-contained and minimal: it carries no
// test-suite integration and is meant to be deleted at campaign close, its learnings kept in docs.

#include "../../libs/SFM.h"
#include <boost/program_options.hpp>

using namespace SFM;


// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("SceneAnalyzeSFM")


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strProject;
String strOutDir;
boost::program_options::variables_map vm;
} // namespace OPT

class Application {
public:
	Application() {}
	~Application() { Finalize(); }

	bool Initialize(size_t argc, LPCTSTR* argv);
	void Finalize();
}; // Application

bool Application::Initialize(size_t argc, LPCTSTR* argv)
{
	OPEN_LOG();
	OPEN_LOGCONSOLE();

	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		;

	boost::program_options::options_description config("Main options");
	config.add_options()
		("project", boost::program_options::value<std::string>(&OPT::strProject), "SFM project file to analyze (the .mvs/scene file CreateStructure writes)")
		("out,o", boost::program_options::value<std::string>(&OPT::strOutDir), "output directory for the CSVs (default: the project's own directory)")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config);

	boost::program_options::positional_options_description p;
	p.add("project", -1);

	try {
		boost::program_options::store(boost::program_options::command_line_parser((int)argc, argv).options(cmdline_options).positional(p).run(), OPT::vm);
		boost::program_options::notify(OPT::vm);
		INIT_WORKING_FOLDER;
	}
	catch (const std::exception& e) {
		LOG(e.what());
		return false;
	}

	// validate input
	Util::ensureValidPath(OPT::strProject);
	if (OPT::vm.count("help") || OPT::strProject.empty()) {
		GET_LOG() << cmdline_options;
		if (OPT::strProject.empty())
			LOG("error: project file is required");
		return false;
	}

	// Resolve the project path and the output directory (--out, or the project's own directory)
	// before opening the log file. This tool's binding constraint is to write nowhere but its
	// output directory; MAKE_PATH()/WORKING_FOLDER (the CreateStructure pattern this was copied
	// from) resolves against the process cwd instead, so the natural invocation
	// (`cd <dataset-dir> && SceneAnalyzeSFM scene.sfm`) would drop the log there rather than
	// beside the CSVs. OPT::strProject/strOutDir are overwritten with their resolved form so main()
	// reuses these instead of re-resolving them.
	OPT::strProject = MAKE_PATH_SAFE(OPT::strProject);
	OPT::strOutDir = OPT::strOutDir.empty() ? Util::getFilePath(OPT::strProject) : MAKE_PATH_SAFE(OPT::strOutDir);
	Util::ensureValidFolderPath(OPT::strOutDir);
	Util::ensureFolder(OPT::strOutDir);

	// initialize the log file inside the resolved output directory, not WORKING_FOLDER
	OPEN_LOGFILE((OPT::strOutDir + APPNAME _T("-") + Util::getUniqueName(0) + _T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	SEACAVE::Initialize(APPNAME);
	return true;
}

void Application::Finalize()
{
	SEACAVE::Finalize();
	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

} // unnamed namespace


// Escape one field for a quoted CSV cell: only '"' needs doubling, the grammar RFC4180 uses.
static String CSVQuote(const String& field)
{
	String out("\"");
	for (char c : field) {
		if (c == '"')
			out += '"';
		out += c;
	}
	out += '"';
	return out;
}

// Write images.csv: one row per image, in array (== ID) order.
// The registered pose and focal travel with the row so the offline campaign scripts never have to
// parse the project file to get them. Conventions, as the project itself stores them (Pose.h, the
// MVS convention P = K*R*[I|-C]): R rotates world coordinates into camera coordinates and is
// written as the unit quaternion (qw,qx,qy,qz) by the very same Pose3DToQuaternionAndCenter the
// bundle adjuster parameterizes poses with, so the CSV cannot drift from the solver's convention;
// (cx,cy,cz) is the camera centre in world units (Pose3D::C, not the translation -R*C); focal is
// the camera's mean focal in pixels (Camera::GetFocalLength). An unregistered image -- no pose or
// no camera -- leaves all eight cells empty rather than emitting an identity pose that a reader
// could mistake for a fitted one. Both branches emit the same NUM_POSE_COLUMNS cells, counted from
// one constant, so that adding a column here cannot desynchronize the empty row from the header.
static bool ExportImagesCSV(const Scene& scene, const String& fileName)
{
	constexpr unsigned NUM_POSE_COLUMNS = 8;   // focal + quaternion (4) + camera centre (3)
	constexpr unsigned NUM_CAMERA_COLUMNS = 4; // image size (2) + principal point (2)
	std::ofstream ofs(fileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", fileName.c_str());
		return false;
	}
	ofs.precision(12); // enough digits that a re-derived pose error is the pose's, not the CSV's
	ofs << "imageID,name,numKeypoints,numDescribedKeypoints,numDenseKeypoints,registered,focal,qw,qx,qy,qz,cx,cy,cz,width,height,ppx,ppy\n";
	FOREACH(idx, scene.images) {
		const Image& img = scene.images[idx];
		const bool bRegistered = img.IsValid();
		ofs << img.ID << ','
			<< CSVQuote(Util::getFileName(img.fileName)) << ','
			<< img.keypoints.size() << ','
			<< img.NumDescribedKeypoints() << ','
			<< img.NumDenseKeypoints() << ','
			<< (bRegistered ? 1 : 0);
		if (bRegistered) {
			double params[NUM_POSE_COLUMNS-1]; // quaternion (w x y z), then the camera centre
			Pose3DToQuaternionAndCenter(img, params);
			ofs << ',' << img.pCamera->GetFocalLength();
			for (const double param : params)
				ofs << ',' << param;
		} else {
			for (unsigned c = 0; c < NUM_POSE_COLUMNS; ++c)
				ofs << ','; // every pose cell of an unregistered image stays empty
		}
		// The camera block is independent of registration: an image that never got a pose still has
		// the resolution its keypoints were detected at, which is what every pixel metric needs.
		if (img.pCamera != NULL) {
			const Point2 pp = img.pCamera->GetPrincipalPoint();
			ofs << ',' << img.pCamera->GetWidth() << ',' << img.pCamera->GetHeight()
				<< ',' << pp.x << ',' << pp.y;
		} else {
			for (unsigned c = 0; c < NUM_CAMERA_COLUMNS; ++c)
				ofs << ',';
		}
		ofs << '\n';
	}
	ofs.close();
	VERBOSE("Exported %u images to '%s'", (unsigned)scene.images.size(), fileName.c_str());
	return true;
}

// One row of matches.bin: where a stored match actually lands in each image, and which keypoint it
// came from. All members are 4 bytes, so the record has no padding on any supported platform and the
// file is a plain numpy-readable array of records.
namespace {
struct MatchRecord {
	float x1, y1, x2, y2; // keypoint pixel coordinates, in image ID1 and image ID2 respectively
	uint32_t idx1, idx2;  // the keypoint indices those coordinates were read from
};
static_assert(sizeof(MatchRecord) == 24, "MatchRecord must stay tightly packed: readers index it as a flat array");
const char MATCHES_MAGIC[8] = {'O','M','V','S','M','T','C','1'};
constexpr uint32_t MATCHES_VERSION = 1;
} // unnamed namespace

// Write pairs.csv and matches.bin: one row per pair, in array order, plus every one of that pair's
// stored matches as pixel coordinates.
// The two files are written by one pass because pairs.csv's matchOffset indexes matches.bin: written
// separately, a mid-pass failure could leave offsets pointing into a file that never got the rows.
// Each pair contributes one contiguous block at matchOffset -- its `matches` array followed by its
// `outlierMatches` array -- so the reader recovers all four segments from the counts in the row:
//   [0, numSparseInliers)                                        sparse, descriptor evidence
//   [.., + numDenseInliers)                                      the ROMAv2 dense supplement
//   [.., + numLooseInliers)                                      RANSAC inliers the strict filter cut
//   [.., + numOutlierMatches)                                    outlierMatches, never track-forming
// compositeWeight is not itself a stored field, but is cheaply derivable from the pair's stored
// weightSpatial/weightConnectivity/weightTriplet components (ImagePair::GetCompositeWeight()), the
// same accessor every view-graph consumer and PairsMatcher::ExportPairsCSV already reads -- so it
// is included rather than omitted.
// The relative pose is written in the convention the pair stores it in and the one poselib handed it
// to FinalizeRelative in: x2 = R*x1 + t, with t = Pose3D::GetT() (not the camera centre). A pair
// without a relative pose leaves all twelve cells empty rather than emitting an identity.
static bool ExportPairsAndMatches(const Scene& scene, const String& pairsFileName, const String& matchesFileName)
{
	constexpr unsigned NUM_POSE_COLUMNS = 12; // rotation (9, row-major) + translation (3)
	std::ofstream ofs(pairsFileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", pairsFileName.c_str());
		return false;
	}
	std::ofstream mfs(matchesFileName, std::ios::binary);
	if (!mfs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", matchesFileName.c_str());
		return false;
	}
	// The total row count is known before the first row is written, so the header never has to be
	// seeked back to and patched -- a truncated file is then detectably short rather than mislabeled.
	uint32_t numRows = 0;
	FOREACH(idx, scene.pairs)
		numRows += (uint32_t)(scene.pairs[idx].matches.size() + scene.pairs[idx].outlierMatches.size());
	const uint32_t version = MATCHES_VERSION;
	mfs.write(MATCHES_MAGIC, sizeof(MATCHES_MAGIC));
	mfs.write((const char*)&version, sizeof(version));
	mfs.write((const char*)&numRows, sizeof(numRows));

	ofs.precision(12); // enough digits that a re-derived pose error is the pose's, not the CSV's
	ofs << "ID1,ID2,numMatches,numSparseInliers,numDenseInliers,numLooseInliers,numOutlierMatches,"
	       "compositeWeight,supplemented,overlapRatio,overlapArea,meanRayAngle,matchOffset,hasPose,"
	       "R00,R01,R02,R10,R11,R12,R20,R21,R22,tx,ty,tz\n";
	uint32_t offset = 0;
	FOREACH(idx, scene.pairs) {
		const ImagePair& pair = scene.pairs[idx];
		const unsigned numSparse = pair.GetNumFilteredInliers();
		const unsigned numDense = pair.GetNumDenseInliers();
		ASSERT(numSparse + numDense <= pair.matches.size());
		const unsigned numLoose = (unsigned)pair.matches.size() - numSparse - numDense;
		const unsigned numOutliers = (unsigned)pair.outlierMatches.size();
		ofs << pair.ID1 << ',' << pair.ID2 << ','
			<< pair.GetNumMatches() << ','
			<< numSparse << ','
			<< numDense << ','
			<< numLoose << ','
			<< numOutliers << ','
			<< pair.GetCompositeWeight() << ','
			<< (numDense > 0 ? 1 : 0) << ','
			<< pair.overlapRatio << ','
			<< pair.overlapArea << ','
			<< pair.meanRayAngle << ','
			<< offset << ','
			<< (pair.relativePose.has_value() ? 1 : 0);
		if (pair.relativePose.has_value()) {
			const Pose3D& rel = pair.relativePose.value();
			const Point3 t = rel.GetT();
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					ofs << ',' << rel.R(r, c);
			ofs << ',' << t.x << ',' << t.y << ',' << t.z;
		} else {
			for (unsigned c = 0; c < NUM_POSE_COLUMNS; ++c)
				ofs << ','; // every pose cell of a pair without a relative pose stays empty
		}
		ofs << '\n';

		// the matches themselves, in stored order, so the segment boundaries above are exact
		const Image& img1 = scene.images[pair.ID1];
		const Image& img2 = scene.images[pair.ID2];
		const auto WriteMatches = [&](const std::vector<DMatch>& arr) {
			for (const DMatch& m : arr) {
				MatchRecord rec;
				rec.idx1 = m.queryIdx;
				rec.idx2 = m.trainIdx;
				if (m.queryIdx < img1.keypoints.size() && m.trainIdx < img2.keypoints.size()) {
					const cv::Point2f& pt1 = img1.keypoints[m.queryIdx].pt;
					const cv::Point2f& pt2 = img2.keypoints[m.trainIdx].pt;
					rec.x1 = pt1.x; rec.y1 = pt1.y;
					rec.x2 = pt2.x; rec.y2 = pt2.y;
				} else {
					// a match indexing a keypoint that is not there is a corrupt project, not a
					// position: the row stays, so offsets keep counting, but it scores nothing
					rec.x1 = rec.y1 = rec.x2 = rec.y2 = std::numeric_limits<float>::quiet_NaN();
				}
				mfs.write((const char*)&rec, sizeof(rec));
			}
		};
		WriteMatches(pair.matches);
		WriteMatches(pair.outlierMatches);
		offset += (uint32_t)(pair.matches.size() + pair.outlierMatches.size());
	}
	ASSERT(offset == numRows);
	ofs.close();
	mfs.close();
	VERBOSE("Exported %u pairs to '%s' and %u matches to '%s'",
		(unsigned)scene.pairs.size(), pairsFileName.c_str(), numRows, matchesFileName.c_str());
	return true;
}

// Write tracks.csv: one row per track, in array (== trackID) order. The observation-refs field is
// sorted by (imageID,featureID) regardless of the track's own internal array order, which
// FilterTracks reorders (inliers first) and so is not itself a stable join key across re-runs.
static bool ExportTracksCSV(const Scene& scene, const String& fileName)
{
	std::ofstream ofs(fileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", fileName.c_str());
		return false;
	}
	ofs << "trackID,length,numDenseObs,numDescribedObs,observations\n";
	FOREACH(trackIdx, scene.tracks) {
		const Track& track = scene.tracks[trackIdx];
		ObservationArr sorted(track.observations);
		sorted.Sort();
		unsigned numDenseObs = 0;
		String refs;
		FOREACH(obsIdx, sorted) {
			const Observation& obs = sorted[obsIdx];
			const bool bDense = obs.imageID < scene.images.size() &&
				obs.featureID < scene.images[obs.imageID].keypoints.size() &&
				scene.images[obs.imageID].IsDenseKeypoint(obs.featureID);
			numDenseObs += bDense;
			if (obsIdx > 0)
				refs += ',';
			refs += String::FormatString("%u:%u", obs.imageID, obs.featureID);
		}
		ofs << trackIdx << ','
			<< track.GetNumObservations() << ','
			<< numDenseObs << ','
			<< (track.GetNumObservations() - numDenseObs) << ','
			<< CSVQuote(refs) << '\n';
	}
	ofs.close();
	VERBOSE("Exported %u tracks to '%s'", (unsigned)scene.tracks.size(), fileName.c_str());
	return true;
}

// Write observations.csv: one row per (track, observation), tracks in array order and each track's
// own observations sorted by (imageID,featureID) -- same ordering as tracks.csv's refs field, so
// the two files can be joined without re-deriving either order.
// Reprojection error reuses SFM::ComputeReprojectionErrorPixels, the same formula FilterTracks
// applies to its kept observations (Track.h), rather than re-deriving the projection here; an
// observation on an unregistered image, or whose projection is behind the camera, leaves the cell
// empty rather than a fabricated number.
static bool ExportObservationsCSV(const Scene& scene, const String& fileName)
{
	std::ofstream ofs(fileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", fileName.c_str());
		return false;
	}
	ofs << "trackID,imageID,featureID,isDense,reprojErrorPixels\n";
	unsigned numRows = 0;
	FOREACH(trackIdx, scene.tracks) {
		const Track& track = scene.tracks[trackIdx];
		ObservationArr sorted(track.observations);
		sorted.Sort();
		for (const Observation& obs : sorted) {
			const bool bValidImage = obs.imageID < scene.images.size();
			const Image* pImg = bValidImage ? &scene.images[obs.imageID] : NULL;
			const bool bValidFeature = pImg && obs.featureID < pImg->keypoints.size();
			const bool bDense = bValidFeature && pImg->IsDenseKeypoint(obs.featureID);
			ofs << trackIdx << ',' << obs.imageID << ',' << obs.featureID << ','
				<< (bDense ? 1 : 0) << ',';
			if (bValidFeature && pImg->IsValid()) {
				const Point3 Xcam = pImg->TransformPointW2C(track.position);
				const auto [pixelError, valid] = ComputeReprojectionErrorPixels(*pImg->pCamera, Xcam, pImg->keypoints[obs.featureID].pt);
				if (valid)
					ofs << pixelError; // else leave the cell empty, matching the "unscored" CSV convention elsewhere
			}
			ofs << '\n';
			++numRows;
		}
	}
	ofs.close();
	VERBOSE("Exported %u observations to '%s'", numRows, fileName.c_str());
	return true;
}


int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index or use _CrtSetBreakAlloc() to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	Application application;
	if (!application.Initialize(argc, argv))
		return EXIT_FAILURE;

	TD_TIMER_START();

	// Load the project read-only; no pipeline stage runs, nothing is written back to it.
	// OPT::strProject/strOutDir were already resolved (and strOutDir's folder ensured to exist)
	// by Initialize(), before it opened the log file inside strOutDir.
	Scene scene(1);
	if (!scene.Load(OPT::strProject)) {
		VERBOSE("error: cannot load project '%s'", OPT::strProject.c_str());
		return EXIT_FAILURE;
	}

	bool ok = true;
	ok &= ExportTracksCSV(scene, OPT::strOutDir + _T("tracks.csv"));
	ok &= ExportObservationsCSV(scene, OPT::strOutDir + _T("observations.csv"));
	ok &= ExportPairsAndMatches(scene, OPT::strOutDir + _T("pairs.csv"), OPT::strOutDir + _T("matches.bin"));
	ok &= ExportImagesCSV(scene, OPT::strOutDir + _T("images.csv"));
	if (!ok)
		return EXIT_FAILURE;

	VERBOSE("Scene analyzed (%s)", TD_TIMER_GET_FMT().c_str());
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
