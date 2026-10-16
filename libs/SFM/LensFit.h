/*
 * LensFit.h
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
 */

#ifndef _SFM_LENSFIT_H_
#define _SFM_LENSFIT_H_

// I N C L U D E S /////////////////////////////////////////////////

#include "Camera.h"


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// Forward declarations
class SFM_API Scene;
class SFM_API PairsMatcher;
struct SFM_API MatchConfig;

/*
 * The radial lens of a camera, estimated from its verified pairs
 * ==============================================================
 *
 * Every pair is verified through its cameras as they are: a pinhole without a lens when none is
 * given. Where the lens is strong (Playground's moves its corners by 72 px), the pairs that turn while
 * barely moving come out degrees short with their baselines reversed, and every reader of the pairs'
 * geometry inherits that. This fits one radial lens per camera that has none -- k1, k2 of the forward
 * model PinholeCamera, OpenCV and COLMAP's OPENCV share, x_d = x_u (1 + k1 r^2 + k2 r^4) on
 * coordinates normalized by the focal around the principal point -- from its pairs, jointly with each
 * pair's relative pose; the principal point is held, and so is the focal unless the focal check below
 * moves it.
 *
 * Which cameras: a pinhole camera that trusts its focal and has no lens is fitted; one whose lens is
 * given as k1, k2 alone, one to one past its image corner, is held, its pairs informing the others;
 * anything else (spherical, focal not trusted, a given lens with more terms, or one that folds within
 * the frame, where its undistortion has no answer) leaves the fit with all its pairs, since held at a
 * wrong lens it would bias every camera it pairs with.
 * What goes in: every verified pair with a pose that turns MatchConfig::lensFitMinRotation to
 * lensFitMaxRotation and holds lensFitMatchesPerPair sparse inliers, between two fitted or held
 * cameras, heaviest first by composite weight until every fitted camera has lensFitMaxPairs; from
 * each, lensFitMatchesPerPair of its sparse inliers, drawn by a generator seeded from the pair's IDs,
 * the same on every platform.
 * A fitted camera with fewer than lensFitMinPairs eligible pairs leaves with its pairs.
 * The loss: the first-order distance of each match's two measured, distorted pixels to the pair's
 * epipolar constraint, each side through its own lens and focal (the constraint's gradient through
 * the inverse lens Jacobian), under soft L1 at lensFitLossScale pixels.
 * The focal check: a focal the pairs clearly reject (an EXIF focal of the full sensor on a cropped frame,
 * one the view-graph calibration biased through the uncorrected lens) is corrected and the lens fitted
 * again at it. The fit as first built runs below with every fitted camera's focal held, its check and
 * restart included; from the lenses and poses it reached, the joint fit and its rounds run again with
 * every fitted camera's focal free beside its lens (a log scale of the focal, a third number of the
 * camera's lens), with the same check and restart; a camera takes the free fit's focal when its median
 * error falls MatchConfig::lensFitFocalMinErrorDrop below the held fit's, and the fit is built again at
 * the moved focal from the free fit's poses. A focal the check keeps leaves the held fit as the fit.
 * In order: the poses alone refined at the starting lenses (their median error is the baseline);
 * the lenses and poses together from the stored poses (each pair's 5 pose unknowns eliminated first,
 * so the reduced system holds 2 unknowns per fitted camera); each pair's pose re-estimated by RANSAC
 * through the fitted lens and kept where it explains the pair better, then the joint fit again, until
 * a round keeps none, for up to lensFitRounds rounds; where that fit fails a check (too few of a fitted
 * camera's pairs keep the fit's median error close to the one each reaches refined alone from the fit,
 * its pose and its own copy of the lenses free, as where the pairs whose pinhole poses the fit could not
 * turn held it short of the lens), all of it again from the fitted lenses doubled, the poses
 * re-estimated through that start first, and the fit of least cost kept; a lens is kept when it stays one
 * to one past the image corner and its median error falls lensFitMinErrorDrop below the baseline, and a
 * fit the solver cannot complete keeps none. A refused camera keeps no lens, and the others are fitted
 * again without its pairs.
 */

// The undistortion displacement of a radial lens (k1, k2 on coordinates normalized by `focal`) at the
// distorted image radius `radius`, both in pixels: radius minus the undistorted radius. Negative for
// a barrel lens (undistortion moves the point outward), positive for a pincushion.
SFM_API REAL LensDisplacement(REAL focal, REAL k1, REAL k2, REAL radius);

// Whether the forward radial map r (1 + k1 r^2 + k2 r^4) rises from the centre until it passes
// `radius` (normalized, > 0): the lens is one to one out to there and its undistortion has one answer
SFM_API bool IsLensOneToOne(REAL k1, REAL k2, REAL radius);

// What the fit decided for one camera
struct SFM_API LensFitOutcome {
	enum Status : uint8_t {
		EXCLUDED = 0,  // not part of the fit: spherical, focal not trusted, or a given lens with terms past k1, k2 or that folds within the frame
		HELD,          // its lens is given as k1, k2 alone and held, its pairs informing the others
		TOO_FEW_PAIRS, // not fitted: fewer eligible pairs than MatchConfig::lensFitMinPairs
		FOLDS,         // refused: the fitted lens is not one to one out to the image corner
		NO_ERROR_DROP, // refused: its median error fell less than MatchConfig::lensFitMinErrorDrop, or the solver failed on its fit
		FITTED         // the lens was kept and set on the camera
	};
	CameraPtr pCamera = NULL;
	Status status = EXCLUDED;
	// the lens, numPairs, the medians and cornerDisplacement describe the last fit that judged the
	// camera, and are 0 where none did (EXCLUDED, TOO_FEW_PAIRS; a HELD camera has its given lens)
	REAL k1 = 0, k2 = 0;           // the lens fitted (FITTED, FOLDS, NO_ERROR_DROP) or given (HELD)
	unsigned numEligiblePairs = 0; // pairs that could inform it when last counted
	unsigned numPairs = 0;         // pairs of the last fit it took part in
	double medianBefore = 0;       // median Sampson error with the poses alone refined at the starting lens, pixels
	double medianAfter = 0;        // the same after the joint fit, pixels
	double cornerDisplacement = 0; // LensDisplacement of the fitted lens at the image corner, pixels; 0 for a lens that folds before it (FOLDS), where undistortion has no answer
	// the focal check, 0 where it did not judge the camera (EXCLUDED, HELD, or not fitted for lack of pairs)
	REAL focalBefore = 0;          // the focal it started from, pixels (the mean of fx, fy)
	REAL focalFree = 0;            // the focal the free fit reached, pixels
	double medianFocalHeld = 0;    // median Sampson error of the fit with the focal held, pixels
	double medianFocalFree = 0;    // the same with the focal free
	bool focalMoved = false;       // the camera took the free fit's focal (fx, fy scaled by focalFree / focalBefore)
};

// What the fit did
struct SFM_API LensFitReport {
	std::vector<LensFitOutcome> cameras;    // one per scene camera, in scene.cameras order
	std::vector<unsigned> reestimatedPoses; // per round of the last fit, from the start it kept: the pairs whose pose the re-estimation replaced
	unsigned numStarts = 0;                 // the joint fits the last fit ran from different starting lenses: 1 where its first passes the check
	double focalSeconds = 0;                // the focal check's share of `seconds`
	double seconds = 0;

	// the cameras whose lens was fitted and set
	std::unordered_set<CameraPtr> FittedCameras() const;
	// the cameras whose pairs were verified at a focal or lens the fit then changed: those whose lens was
	// fitted and set, and those whose focal the check moved, whether or not a lens was kept at it
	std::unordered_set<CameraPtr> CamerasToVerifyAgain() const;
};

// Fit the radial lens of every pinhole camera that has none and trusts its focal, from the scene's
// verified pairs, after checking the focal of each such camera: set every focal moved on its camera
// (fx and fy scaled alike) and every lens kept (SetDistortion(k1, k2, 0, 0); the trust flag is left as
// it is). The pairs themselves are not changed.
SFM_API LensFitReport FitLensesFromPairs(Scene& scene, const MatchConfig& config);

// The lens stage of the matching: FitLensesFromPairs, then every pair of a camera whose lens was
// fitted or whose focal moved (LensFitReport::CamerasToVerifyAgain) verified again through its focal
// and lens (PairsMatcher::ComputeRelativePoses: a descriptor pair through
// GeometricFilter, its outliers rejoining the candidates; a RoMa2 pair through ReverifyPairROMA2,
// its sparse and dense segments kept apart), which also recomputes the pair weights. Logs every
// camera's outcome, the pairs' inliers before and after, and how many of the pairs that formed tracks
// the verification invalidated.
SFM_API LensFitReport FitLensesAndReverifyPairs(PairsMatcher& pairsMatcher);

} // namespace SFM

#endif // _SFM_LENSFIT_H_
