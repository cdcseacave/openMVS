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
 * pair's relative pose; focal and principal point are held.
 *
 * Which cameras: a pinhole camera that trusts its focal and has no lens is fitted; one whose lens is
 * given as k1, k2 alone is held, its pairs informing the others; anything else (spherical, focal not
 * trusted, a given lens with more terms) leaves the fit with all its pairs, since held at a wrong
 * lens it would bias every camera it pairs with.
 * What goes in: every verified pair with a pose that turns MatchConfig::lensFitMinRotation to
 * lensFitMaxRotation and holds lensFitMatchesPerPair sparse inliers, between two fitted or held
 * cameras, heaviest first by composite weight until every fitted camera has lensFitMaxPairs; from
 * each, lensFitMatchesPerPair of its sparse inliers, drawn by a generator seeded from the pair's IDs.
 * A fitted camera with fewer than lensFitMinPairs eligible pairs leaves with its pairs.
 * The loss: the first-order distance of each match's two measured, distorted pixels to the pair's
 * epipolar constraint, each side through its own lens and focal (the constraint's gradient through
 * the inverse lens Jacobian), under soft L1 at lensFitLossScale pixels.
 * In order: the poses alone refined at the starting lenses (their median error is the baseline);
 * the lenses and poses together from the stored poses (each pair's 5 pose unknowns eliminated first,
 * so the reduced system holds 2 unknowns per fitted camera); each pair's pose re-estimated by RANSAC
 * through the fitted lens and kept where it explains the pair better, then the joint fit again, for up
 * to lensFitRounds rounds; a lens is kept when it stays one to one past the image corner and its
 * median error falls lensFitMinErrorDrop below the baseline. A refused camera keeps no lens, and the
 * others are fitted again without its pairs.
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
		EXCLUDED = 0,  // not part of the fit: spherical, focal not trusted, or a given lens with terms past k1, k2
		HELD,          // its lens is given as k1, k2 alone and held, its pairs informing the others
		TOO_FEW_PAIRS, // not fitted: fewer eligible pairs than MatchConfig::lensFitMinPairs
		FOLDS,         // refused: the fitted lens is not one to one out to the image corner
		NO_ERROR_DROP, // refused: its median error fell less than MatchConfig::lensFitMinErrorDrop
		FITTED         // the lens was kept and set on the camera
	};
	CameraPtr pCamera = NULL;
	Status status = EXCLUDED;
	REAL k1 = 0, k2 = 0;           // the lens fitted (FITTED, FOLDS, NO_ERROR_DROP) or given (HELD)
	unsigned numEligiblePairs = 0; // pairs that could inform it when last counted
	unsigned numPairs = 0;         // pairs of the last fit it took part in
	double medianBefore = 0;       // median Sampson error with the poses alone refined at the starting lens, pixels
	double medianAfter = 0;        // the same after the joint fit, pixels
	double cornerDisplacement = 0; // LensDisplacement of the fitted lens at the image corner, pixels
};

// What the fit did
struct SFM_API LensFitReport {
	std::vector<LensFitOutcome> cameras;    // one per scene camera, in scene.cameras order
	std::vector<unsigned> reestimatedPoses; // per round of the last fit: the pairs whose pose the re-estimation replaced
	double seconds = 0;

	// the cameras whose lens was fitted and set
	std::unordered_set<CameraPtr> FittedCameras() const;
};

// Fit the radial lens of every pinhole camera that has none and trusts its focal, from the scene's
// verified pairs, and set every lens kept on its camera (SetDistortion(k1, k2, 0, 0); the trust flag
// is left as it is). The pairs themselves are not changed.
SFM_API LensFitReport FitLensesFromPairs(Scene& scene, const MatchConfig& config);

} // namespace SFM

#endif // _SFM_LENSFIT_H_
