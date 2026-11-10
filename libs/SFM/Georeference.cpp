// CRS-aware camera priors and ground control, optimized in a shared local ENU frame.
#include "Common.h"
#include "Georeference.h"
#include "../Math/GeodeticTransforms.h"
#include <proj.h>
#include <Eigen/Cholesky>

using namespace SFM;

namespace {
class CoordinateTransform {
	std::unique_ptr<PJ_CONTEXT, decltype(&proj_context_destroy)> context{proj_context_create(), proj_context_destroy};
	std::unique_ptr<PJ, decltype(&proj_destroy)> operation{nullptr, proj_destroy};
public:
	CoordinateTransform(const String& source, const String& target) {
		ASSERT(!source.empty() && !target.empty());
		proj_context_set_enable_network(context.get(), 0);
		std::unique_ptr<PJ, decltype(&proj_destroy)> src(proj_create(context.get(), source.c_str()), proj_destroy);
		std::unique_ptr<PJ, decltype(&proj_destroy)> dst(proj_create(context.get(), target.c_str()), proj_destroy);
		const char* options[] = {"ALLOW_BALLPARK=NO", "ONLY_BEST=YES", nullptr};
		if (src && dst) {
			std::unique_ptr<PJ, decltype(&proj_destroy)> raw(
				proj_create_crs_to_crs_from_pj(context.get(), src.get(), dst.get(), nullptr, options), proj_destroy);
			if (raw)
				operation.reset(proj_normalize_for_visualization(context.get(), raw.get()));
		}
		if (!operation)
			VERBOSE("error: cannot transform CRS '%s' to '%s'", source.c_str(), target.c_str());
	}
	bool IsValid() const { return bool(operation); }
	bool Apply(const Point3& input, Point3& output) const {
		ASSERT(operation);
		proj_errno_reset(operation.get());
		const PJ_COORD result = proj_trans(operation.get(), PJ_FWD, proj_coord(input.x, input.y, input.z, HUGE_VAL));
		output = Point3(result.xyz.x, result.xyz.y, result.xyz.z);
		if (proj_errno(operation.get()) || !ISFINITE(output)) {
			VERBOSE("error: CRS transformation failed; check coordinates, height datum and installed PROJ grids");
			return false;
		}
		return true;
	}
};

std::vector<String> SplitCSV(const String& line) {
	std::vector<String> fields;
	std::stringstream stream(line);
	String field;
	while (std::getline(stream, field, ',')) {
		if (!field.empty() && field.back() == '\r') field.pop_back();
		fields.push_back(field);
	}
	return fields;
}

double ParseNumber(const String& value) {
	size_t end;
	const double result = std::stod(value, &end);
	while (end < value.size() && std::isspace(static_cast<unsigned char>(value[end]))) ++end;
	if (end != value.size()) throw std::invalid_argument("invalid numeric field");
	return result;
}
}

bool SFM::ReprojectCoordinates(const String& sourceCRS, const String& targetCRS,
	const Point3Arr& input, Point3Arr& output)
{
	ASSERT(!sourceCRS.empty() && !targetCRS.empty());
	CoordinateTransform conversion(sourceCRS, targetCRS);
	if (!conversion.IsValid()) return false;
	Point3Arr converted;
	for (const Point3& point : input) {
		Point3 result;
		if (!conversion.Apply(point, result)) return false;
		converted.push_back(result);
	}
	output = std::move(converted);
	return true;
}

Matrix4x4 SFM::ENUToECEFTransform(const Point3& originECEF)
{
	ASSERT(ISFINITE(originECEF) && norm(originECEF) > 1);
	double lat, lon, alt;
	ECEFToWGS84(originECEF.x, originECEF.y, originECEF.z, lat, lon, alt);
	Matrix4x4 result(Matrix4x4::IDENTITY);
	for (unsigned axis = 0; axis < 3; ++axis) {
		Point3 unit(Point3::ZERO), endpoint;
		unit[axis] = 1;
		ENUToECEF(unit.x, unit.y, unit.z, originECEF.x, originECEF.y, originECEF.z,
			lat, lon, endpoint.x, endpoint.y, endpoint.z);
		for (unsigned row = 0; row < 3; ++row)
			result(row, axis) = endpoint[row]-originECEF[row];
	}
	for (unsigned row = 0; row < 3; ++row)
		result(row, 3) = originECEF[row];
	return result;
}

bool SFM::ImportGPSCSV(const String& path, const String& crs, ImageArr& images)
{
	ASSERT(!path.empty() && !crs.empty());
	std::ifstream stream(path);
	String line;
	if (!stream || !std::getline(stream, line)) {
		VERBOSE("error: cannot read GPS CSV '%s'", path.c_str());
		return false;
	}
	const auto header = SplitCSV(line);
	std::unordered_map<String, size_t> columns;
	for (size_t i = 0; i < header.size(); ++i) columns.emplace(header[i], i);
	for (const char* name : {"filename", "x", "y", "z", "accuracy_xy", "accuracy_z"}) {
		if (!columns.count(name)) {
			VERBOSE("error: GPS CSV missing column '%s'", name);
			return false;
		}
	}
	std::unordered_map<String, IIndex> imageIDs;
	FOREACH(i, images) {
		if (!imageIDs.emplace(Util::getFileName(images[i].fileName), i).second) {
			VERBOSE("error: GPS CSV requires unique image filename stems");
			return false;
		}
	}
	CoordinateTransform conversion(crs, "EPSG:4979");
	if (!conversion.IsValid()) return false;
	std::unordered_set<IIndex> seen;
	unsigned row = 1;
	while (std::getline(stream, line)) {
		++row;
		if (line.empty()) continue;
		try {
			const auto fields = SplitCSV(line);
			const auto field = [&](const char* name) -> const String& { return fields.at(columns.at(name)); };
			const auto image = imageIDs.find(Util::getFileName(field("filename")));
			if (image == imageIDs.end()) continue;
			const Point3 position(ParseNumber(field("x")), ParseNumber(field("y")), ParseNumber(field("z")));
			const double horizontal = ParseNumber(field("accuracy_xy")), vertical = ParseNumber(field("accuracy_z"));
			if (!ISFINITE(position) || !std::isfinite(horizontal) || horizontal <= 0 ||
				!std::isfinite(vertical) || vertical <= 0 || seen.count(image->second))
				throw std::invalid_argument("nonfinite coordinates, invalid accuracy or duplicate image");
			Point3 lla;
			if (!conversion.Apply(position, lla)) return false;
			if (ABS(lla.x) > 180 || ABS(lla.y) > 90)
				throw std::invalid_argument("longitude/latitude outside valid range");
			auto& metadata = images[image->second].View::metadata;
			metadata.longitude = lla.x;
			metadata.latitude = lla.y;
			metadata.altitude = lla.z;
			metadata.positionAccuracy = horizontal;
			metadata.positionAccuracyZ = vertical;
			seen.insert(image->second);
		} catch (const std::exception& error) {
			VERBOSE("error: GPS CSV row %u: %s", row, error.what());
			return false;
		}
	}
	if (seen.empty()) {
		VERBOSE("error: no matching camera positions in GPS CSV '%s'", path.c_str());
		return false;
	}
	VERBOSE("Imported %u camera GPS priors from '%s' (%s)", (unsigned)seen.size(), path.c_str(), crs.c_str());
	return true;
}

bool SFM::PrepareGCPReference(const Scene& scene, Point3Arr& targets,
	std::vector<Matrix3x3>& whitening, Point3& originECEF)
{
	ASSERT(!scene.gcps.empty());
	for ([[maybe_unused]] const auto& gcp : scene.gcps) {
		ASSERT(ISFINITE(gcp.position) && ISFINITE(gcp.accuracy));
		ASSERT(gcp.accuracy.x > 0 && gcp.accuracy.y > 0 && gcp.accuracy.z > 0);
	}
	targets.clear();
	whitening.clear();
	if (scene.gcpCRS.empty()) {
		for (const auto& gcp : scene.gcps) {
			targets.push_back(gcp.position);
			whitening.emplace_back(1/gcp.accuracy.x, 0, 0, 0, 1/gcp.accuracy.y, 0, 0, 0, 1/gcp.accuracy.z);
		}
		return true;
	}
	CoordinateTransform conversion(scene.gcpCRS, "EPSG:4978");
	if (!conversion.IsValid()) return false;
	Point3Arr ecef;
	originECEF = Point3::ZERO;
	for (const auto& gcp : scene.gcps) {
		Point3 point;
		if (!conversion.Apply(gcp.position, point)) return false;
		ecef.push_back(point);
		originECEF += point;
	}
	originECEF /= REAL(ecef.size());
	double lat, lon, alt;
	ECEFToWGS84(originECEF.x, originECEF.y, originECEF.z, lat, lon, alt);
	const auto ToENU = [&](const Point3& point) {
		Point3 result;
		ECEFToENU(point.x, point.y, point.z, originECEF.x, originECEF.y, originECEF.z, lat, lon,
			result.x, result.y, result.z);
		return result;
	};
	FOREACH(i, scene.gcps) {
		const auto& gcp = scene.gcps[i];
		targets.push_back(ToENU(ecef[i]));
		// Propagate the source-coordinate covariance, retaining correlations caused by rotation.
		Eigen::Matrix3d jacobian;
		for (unsigned axis = 0; axis < 3; ++axis) {
			const double step = MAXF(ABS(gcp.position[axis])*1e-8, 1e-5);
			Point3 plus = gcp.position, minus = gcp.position, a, b;
			plus[axis] += step;
			minus[axis] -= step;
			if (!conversion.Apply(plus, a) || !conversion.Apply(minus, b)) return false;
			const Point3 derivative((ToENU(a)-ToENU(b))/(2*step));
			jacobian.col(axis) = Point3::EVec(derivative);
		}
		const Eigen::Vector3d sigma(gcp.accuracy.x, gcp.accuracy.y, gcp.accuracy.z);
		const Eigen::Matrix3d covariance = jacobian * sigma.array().square().matrix().asDiagonal() * jacobian.transpose();
		const Eigen::LLT<Eigen::Matrix3d> factor(covariance);
		if (factor.info() != Eigen::Success) {
			VERBOSE("error: GCP '%s' has a singular transformed covariance", gcp.label.c_str());
			return false;
		}
		const Eigen::Matrix3d inverse = factor.matrixL().solve(Eigen::Matrix3d::Identity());
		whitening.emplace_back(inverse);
	}
	return true;
}
