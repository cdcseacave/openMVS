/*
 * PoseLink.h
 *
 * Copyright (c) 2014-2025 SEACAVE
 */

#ifndef _SFM_POSELINK_H_
#define _SFM_POSELINK_H_


// I N C L U D E S /////////////////////////////////////////////////

#include "Image.h"
#include "ImagePair.h"


// D E F I N E S ///////////////////////////////////////////////////


// S T R U C T S ///////////////////////////////////////////////////

namespace SFM {

// A verified pair (a known relative pose and a valid weight) joining an image to another one, with
// the pair's relative pose turned around so that
//   pose(image) = relPose * pose(neighbor)
// holds whichever side of the pair the neighbor is on. ImagePair stores the relative pose as the
// transform from ID1 to ID2, so it is used as it is when the neighbor is ID1 and inverted when the
// neighbor is ID2 -- the same turn-around the star initializer applies around its reference view.
struct PoseLink
{
	IIndex neighborID;   // image on the other side of the pair
	unsigned numInliers; // weighted inliers of the pair
	Pose3D relPose;      // transform from the neighbor's frame to the image's frame

	// The absolute rotation this link predicts for the image, given its neighbor's own rotation
	RMatrix PredictedRotation(const Pose3D& neighborPose) const {
		return RMatrix(relPose.R * neighborPose.R);
	}

	// World-frame unit direction from the neighbor's center towards the image's center: relPose.C is
	// R_neighbor * (C_image - C_neighbor), so rotating it back into the world frame gives that
	// direction, while its length is the pair's own (usually arbitrary) scale. False when the pair
	// has no baseline at all, in which case it fixes no direction.
	bool PredictedDirection(const Pose3D& neighborPose, Point3& direction) const {
		direction = neighborPose.R.t() * relPose.C;
		const REAL length = norm(direction);
		if (length <= ZEROTOLERANCE<REAL>())
			return false;
		direction /= length;
		return true;
	}
};
/*----------------------------------------------------------------*/

// Does this pair join its two images with a usable relative pose?
inline bool IsPoseLinkPair(const ImagePair& pair)
{
	return pair.relativePose.has_value() && pair.HasValidWeight();
}

// The link the given pair makes, seen from the image on the other side of the given neighbor
inline PoseLink MakePoseLink(const ImagePair& pair, IIndex neighborID)
{
	ASSERT(pair.relativePose.has_value() && (pair.ID1 == neighborID || pair.ID2 == neighborID));
	return PoseLink{ neighborID, pair.GetNumWeightedInliers(),
		pair.ID1 == neighborID ? pair.relativePose.value() : pair.relativePose->Inverse() };
}

// How far a verified pair's word about an image sits from the pose a model gives it: the angle
// between the rotation the pair predicts from the neighbor's pose and the image's own, and the
// angle between the baseline direction the pair predicts and the one the two centres make. The
// direction is left unset (negative) when the pair fixes none: a near-duplicate pair (its matches
// triangulate under two degrees, so its own baseline direction is unreliable), a pair of no
// baseline, or two centres that coincide; such a pair is judged on the rotation alone.
// The direction is held to the tolerance as seen from the structure the pair triangulates, not
// from the neighbor: a pose off by the tolerance angle as seen from that structure, which sits at
// about baseline / rayAngle, swings the baseline direction by atan(tan(tolerance) / rayAngle), so
// a pair of nearly parallel rays forgives a wide direction error and a wide-baseline pair little.
// A pair of unknown ray angle is held to the plain tolerance.
struct PairDisagreement
{
	REAL rotation;  // degrees
	REAL direction; // degrees; negative when the pair fixes no direction
	REAL rayAngle;  // radians, the pair's median triangulation angle; 0 when unknown
	bool HasDirection() const { return direction >= 0; }
	REAL DirectionTolerance(float maxAngle) const {
		return rayAngle > 0 ? R2D(ATAN(TAN(D2R(REAL(maxAngle))) / rayAngle)) : REAL(maxAngle);
	}
	bool Within(float maxAngle) const {
		return rotation <= maxAngle && (!HasDirection() || direction <= DirectionTolerance(maxAngle));
	}
};
inline PairDisagreement MeasurePairDisagreement(const ImagePair& pair, const Pose3D& image, const Pose3D& neighbor, IIndex neighborID)
{
	const PoseLink link = MakePoseLink(pair, neighborID);
	PairDisagreement d;
	d.rotation = R2D(ACOS(MINF(MAXF(ComputeAngle(image.R, link.PredictedRotation(neighbor)), REAL(-1)), REAL(1))));
	d.direction = -1;
	d.rayAngle = ISFINITE(pair.meanRayAngle) && pair.meanRayAngle > 0.f ? REAL(pair.meanRayAngle) : REAL(0);
	const bool nearDuplicate = pair.meanRayAngle > 0.f && pair.meanRayAngle < D2R(2.f);
	Point3 modelDirection(image.C - neighbor.C), pairDirection;
	const REAL baseline = norm(modelDirection);
	if (!nearDuplicate && baseline > ZEROTOLERANCE<REAL>() && link.PredictedDirection(neighbor, pairDirection))
		d.direction = R2D(ACOS(MINF(MAXF(pairDirection.dot(modelDirection / baseline), REAL(-1)), REAL(1))));
	return d;
}

// Order links by pair strength, ties by the lower neighbor ID so the choice never depends on the
// order the pairs happen to be stored in
inline bool IsStrongerLink(const PoseLink& a, const PoseLink& b)
{
	return a.numInliers > b.numInliers || (a.numInliers == b.numInliers && a.neighborID < b.neighborID);
}
/*----------------------------------------------------------------*/

// The largest group of an image's links that agree about how the image is oriented. Every link
// predicts an absolute rotation for the image (its relative rotation composed with its neighbor's);
// two links agree when their predictions are within a given angle of each other, and the group
// carrying the most weighted inliers wins. One link is one witness, and a link to a misplaced
// neighbor is a false one, so a rule that trusts the single strongest link follows whichever
// neighbor happens to be wrong, while a rule that trusts the largest agreeing group needs the wrong
// neighbors to outweigh the right ones.
struct PoseLinkQuorum
{
	std::vector<PoseLink> links; // the agreeing links, strongest first; empty only when there is none
	RMatrix R;                   // the rotation the strongest of them predicts for the image
	unsigned numConsidered;      // links the quorum was chosen among (the image's strongest ones)
	double maxDisagreement;      // largest angle (degrees) between two of the considered predictions

	// Do the links contradict one another, i.e. is there not even a pair of agreeing ones?
	bool IsContested() const { return numConsidered >= 2 && links.size() < 2; }
};

// Quorum over the strongest maxLinks of the given links, which must be sorted strongest first.
// neighborPose(neighborID) returns the absolute pose of a link's neighbor. maxAngle <= 0 turns the
// clustering off, leaving every considered link in the quorum.
template <typename TNeighborPose>
PoseLinkQuorum ComputePoseLinkQuorum(const std::vector<PoseLink>& links, float maxAngle,
	const TNeighborPose& neighborPose, unsigned maxLinks = 5)
{
	PoseLinkQuorum quorum;
	quorum.numConsidered = MINF((unsigned)links.size(), maxLinks);
	quorum.maxDisagreement = 0;
	if (quorum.numConsidered == 0)
		return quorum;
	// what each of the strongest links says the image's rotation is
	std::vector<RMatrix> predicted;
	predicted.reserve(quorum.numConsidered);
	for (unsigned i = 0; i < quorum.numConsidered; ++i)
		predicted.push_back(links[i].PredictedRotation(neighborPose(links[i].neighborID)));
	if (maxAngle <= 0.f) {
		quorum.links.assign(links.begin(), links.begin() + quorum.numConsidered);
		quorum.R = predicted.front();
		return quorum;
	}
	// Group the predictions around each of them in turn and keep the heaviest group. The links come
	// strongest first and a tie keeps the group found first, so a tie goes to the group holding the
	// strongest link.
	const REAL minCosAngle = COS(D2R(REAL(maxAngle)));
	std::vector<unsigned> members, bestMembers;
	unsigned bestInliers = 0;
	for (unsigned i = 0; i < quorum.numConsidered; ++i) {
		members.clear();
		unsigned numInliers = 0;
		for (unsigned j = 0; j < quorum.numConsidered; ++j) {
			const REAL cosAngle = ComputeAngle(predicted[i], predicted[j]);
			if (i < j)
				quorum.maxDisagreement = MAXF(quorum.maxDisagreement, (double)R2D(ACOS(cosAngle)));
			if (cosAngle >= minCosAngle) {
				members.push_back(j);
				numInliers += links[j].numInliers;
			}
		}
		if (bestMembers.empty() || numInliers > bestInliers) {
			bestInliers = numInliers;
			bestMembers.swap(members);
		}
	}
	quorum.links.reserve(bestMembers.size());
	for (const unsigned j : bestMembers)
		quorum.links.push_back(links[j]);
	quorum.R = predicted[bestMembers.front()];
	return quorum;
}
/*----------------------------------------------------------------*/

} // namespace SFM

#endif // _SFM_POSELINK_H_
