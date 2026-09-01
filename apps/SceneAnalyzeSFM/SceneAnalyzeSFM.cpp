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
// one CSV per entity (tracks, observations, pairs, images) beside it -- no pipeline change, nothing
// is written back into the project. Deliberately self-contained and minimal: it carries no
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

	// initialize the log file
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-") + Util::getUniqueName(0) + _T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strProject);
	if (OPT::vm.count("help") || OPT::strProject.empty()) {
		GET_LOG() << cmdline_options;
		if (OPT::strProject.empty())
			LOG("error: project file is required");
		return false;
	}
	Util::ensureValidFolderPath(OPT::strOutDir);

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
static bool ExportImagesCSV(const Scene& scene, const String& fileName)
{
	std::ofstream ofs(fileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", fileName.c_str());
		return false;
	}
	ofs << "imageID,name,numKeypoints,numDescribedKeypoints,numDenseKeypoints,registered\n";
	FOREACH(idx, scene.images) {
		const Image& img = scene.images[idx];
		ofs << img.ID << ','
			<< CSVQuote(Util::getFileName(img.fileName)) << ','
			<< img.keypoints.size() << ','
			<< img.NumDescribedKeypoints() << ','
			<< img.NumDenseKeypoints() << ','
			<< (img.IsValid() ? 1 : 0) << '\n';
	}
	ofs.close();
	VERBOSE("Exported %u images to '%s'", (unsigned)scene.images.size(), fileName.c_str());
	return true;
}

// Write pairs.csv: one row per pair, in array order.
// compositeWeight is not itself a stored field, but is cheaply derivable from the pair's stored
// weightSpatial/weightConnectivity/weightTriplet components (ImagePair::GetCompositeWeight()), the
// same accessor every view-graph consumer and PairsMatcher::ExportPairsCSV already reads -- so it
// is included rather than omitted.
static bool ExportPairsCSV(const Scene& scene, const String& fileName)
{
	std::ofstream ofs(fileName);
	if (!ofs.is_open()) {
		VERBOSE("error: cannot open file '%s' for writing", fileName.c_str());
		return false;
	}
	ofs << "ID1,ID2,numMatches,numSparseInliers,numDenseInliers,compositeWeight,supplemented\n";
	FOREACH(idx, scene.pairs) {
		const ImagePair& pair = scene.pairs[idx];
		const unsigned numDenseInliers = pair.GetNumDenseInliers();
		ofs << pair.ID1 << ',' << pair.ID2 << ','
			<< pair.GetNumMatches() << ','
			<< pair.GetNumFilteredInliers() << ','
			<< numDenseInliers << ','
			<< pair.GetCompositeWeight() << ','
			<< (numDenseInliers > 0 ? 1 : 0) << '\n';
	}
	ofs.close();
	VERBOSE("Exported %u pairs to '%s'", (unsigned)scene.pairs.size(), fileName.c_str());
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

	// Load the project read-only; no pipeline stage runs, nothing is written back to it
	const String projectPath(MAKE_PATH_SAFE(OPT::strProject));
	Scene scene(1);
	if (!scene.Load(projectPath)) {
		VERBOSE("error: cannot load project '%s'", projectPath.c_str());
		return EXIT_FAILURE;
	}

	// Default output directory is the project's own directory
	String outDir(OPT::strOutDir.empty() ? Util::getFilePath(projectPath) : MAKE_PATH_SAFE(OPT::strOutDir));
	Util::ensureValidFolderPath(outDir);
	Util::ensureFolder(outDir);

	bool ok = true;
	ok &= ExportTracksCSV(scene, outDir + _T("tracks.csv"));
	ok &= ExportObservationsCSV(scene, outDir + _T("observations.csv"));
	ok &= ExportPairsCSV(scene, outDir + _T("pairs.csv"));
	ok &= ExportImagesCSV(scene, outDir + _T("images.csv"));
	if (!ok)
		return EXIT_FAILURE;

	VERBOSE("Scene analyzed (%s)", TD_TIMER_GET_FMT().c_str());
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
