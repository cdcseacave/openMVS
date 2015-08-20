////////////////////////////////////////////////////////////////////
// OBJ.cpp
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#include "Common.h"
#include "OBJ.h"

using namespace SEACAVE;


// D E F I N E S ///////////////////////////////////////////////////

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define OBJ_USE_OPENMP
#endif

#define OBJ_INDEX_OFFSET 1


// S T R U C T S ///////////////////////////////////////////////////

ObjModel::MaterialLib::Material::Material(const Image8U3& _diffuse_map, const Color& _Kd)
	:
	diffuse_map(_diffuse_map),
	Kd(_Kd)
{
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

ObjModel::MaterialLib::MaterialLib()
{
}

void ObjModel::MaterialLib::AddMaterial(String const & name, Material material)
{
	material_names.push_back(name);
	materials.push_back(material);
}

bool ObjModel::MaterialLib::Save(const String& prefix) const
{
	String filename(prefix + ".mtl");
	std::ofstream out(filename.c_str());
	if (!out.good())
		return false;

	String name(Util::getFileName(prefix));

	for (size_t i = 0; i < materials.size(); ++i) {
		const Material& mat = materials[i];
		String diffuse_map_postfix("_" + material_names[i] + "_map_Kd.png");
		out << "newmtl " << material_names[i] << std::endl
			<< "Ka 1.000000 1.000000 1.000000" << std::endl
			<< "Kd " << mat.Kd.r << " " << mat.Kd.g << " " << mat.Kd.b << std::endl
			<< "Ks 0.000000 0.000000 0.000000" << std::endl
			<< "Tr 1.000000" << std::endl
			<< "illum 1" << std::endl
			<< "Ns 1.000000" << std::endl
			<< "map_Kd " << name + diffuse_map_postfix << std::endl;
	}
	out.close();

	#ifdef OBJ_USE_OPENMP
	bool bSuccess(true);
	#pragma omp parallel for
	#endif
	for (int_t i = 0; i < (int_t)materials.size(); ++i) {
		String diffuse_map_postfix("_" + material_names[i] + "_map_Kd.png");
		const bool bRet(materials[i].diffuse_map.Save(prefix + diffuse_map_postfix));
		#ifdef OBJ_USE_OPENMP
		#pragma omp critical
		if (!bRet)
			bSuccess = false;
		#else
		if (!bRet)
			return false;
		#endif
	}
	#ifdef OBJ_USE_OPENMP
	return bSuccess;
	#else
	return true;
	#endif
}

bool ObjModel::MaterialLib::Load(const String& fileName)
{
	std::ifstream in(fileName.c_str());
	if (!in.good())
		return false;

	String keyword;
	while (!in.eof()) {
		in >> keyword;
		if (keyword == "newmtl") {
			in >> keyword;
			material_names.push_back(keyword);
			materials.push_back(Material());
		} else if (keyword == "Kd") {
			Color c;
			in >> c.r >> c.g >> c.b;
			materials.back().Kd = c;
		} else if (keyword == "map_Kd") {
			in >> keyword;
			materials.back().diffuse_map.Load(keyword);
		}
	}
	return true;
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

ObjModel::ObjModel()
{
}

bool ObjModel::Save(const String& prefix) const
{
	if (!material_lib.Save(prefix))
		return false;

	String name(Util::getFileName(prefix));
	std::ofstream out((prefix + ".obj").c_str());
	if (!out.good())
		return false;

	out << "mtllib " << name << ".mtl" << std::endl;

	out << std::fixed << std::setprecision(6);
	for (size_t i = 0; i < vertices.size(); ++i) {
		out << "v " << vertices[i][0] << " "
			<< vertices[i][1] << " "
			<< vertices[i][2] << std::endl;
	}

	for (size_t i = 0; i < texcoords.size(); ++i) {
		out << "vt " << texcoords[i][0] << " "
			<< texcoords[i][1] << std::endl;
	}

	for (size_t i = 0; i < normals.size(); ++i) {
		out << "vn " << normals[i][0] << " "
			<< normals[i][1] << " "
			<< normals[i][2] << std::endl;
	}

	for (size_t i = 0; i < groups.size(); ++i) {
		out << "usemtl " << groups[i].material_name << std::endl;
		for (size_t j = 0; j < groups[i].faces.size(); ++j) {
			Face const & face =  groups[i].faces[j];
			out << "f";
			for (size_t k = 0; k < 3; ++k) {
				out << " " << face.vertices[k]  + OBJ_INDEX_OFFSET
					<< "/" << face.texcoords[k]  + OBJ_INDEX_OFFSET
					<< "/" << face.normals[k]  + OBJ_INDEX_OFFSET;
			}
			out << std::endl;
		}
	}
	return true;
}

bool ObjModel::Load(const String& fileName)
{
	std::ifstream in(fileName.c_str());
	if (!in.good())
		return false;

	String keyword;
	in >> keyword;
	if (keyword != "mtllib")
		return false;
	in >> keyword;
	if (!material_lib.Load(keyword))
		return false;

	in >> keyword;
	while (keyword == "v") {
		Vec3f v;
		in >> v[0] >> v[1] >> v[2];
		if (!in.good())
			return false;
		vertices.push_back(v);
		in >> keyword;
	}
	while (keyword == "vt") {
		Vec2f vt;
		in >> vt[0] >> vt[1];
		if (!in.good())
			return false;
		texcoords.push_back(vt);
		in >> keyword;
	}
	while (keyword == "vn") {
		Vec3f vn;
		in >> vn[0] >> vn[1] >> vn[2];
		if (!in.good())
			return false;
		normals.push_back(vn);
		in >> keyword;
	}
	while (keyword == "usemtl") {
		Group group;
		in >> group.material_name;
		if (!in.good())
			return false;
		in >> keyword;
		while (keyword == "f") {
			Face f;
			for (size_t k = 0; k < 3; ++k) {
				in >> keyword;
				sscanf(keyword, "%zu/%zu/%zu", f.vertices+k, f.texcoords+k, f.normals+k);
				f.vertices[k] -= OBJ_INDEX_OFFSET;
				f.texcoords[k] -= OBJ_INDEX_OFFSET;
				f.normals[k] -= OBJ_INDEX_OFFSET;
			}
			if (!in.good())
				return false;
			group.faces.push_back(f);
			in >> keyword;
		}
		groups.push_back(group);
	}
	return true;
}
/*----------------------------------------------------------------*/
