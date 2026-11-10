#ifndef _SFM_GEOREFERENCE_H_
#define _SFM_GEOREFERENCE_H_

#include "Scene.h"

namespace SFM {

// CRS coordinates use visualization order: longitude/latitude or easting/northing.
SFM_API bool ReprojectCoordinates(const String& sourceCRS, const String& targetCRS,
	const Point3Arr& input, Point3Arr& output);
SFM_API bool ImportGPSCSV(const String& path, const String& crs, ImageArr& images);
SFM_API Matrix4x4 ENUToECEFTransform(const Point3& originECEF);

// Survey coordinates remain unchanged; outputs are BA targets and inverse covariance factors.
SFM_API bool PrepareGCPReference(const Scene& scene, Point3Arr& targets,
	std::vector<Matrix3x3>& whitening, Point3& originECEF);

}
#endif
