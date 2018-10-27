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

bool ObjModel::MaterialLib::Material::LoadDiffuseMap()
{
	if (diffuse_map.empty())
		return diffuse_map.Load(diffuse_name);
	return true;
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

ObjModel::MaterialLib::MaterialLib()
{
}

bool ObjModel::MaterialLib::Save(const String& prefix, bool texLossless) const
{
	std::ofstream out((prefix+".mtl").c_str());
	if (!out.good())
		return false;

	const String pathName(Util::getFilePath(prefix));
	const String name(Util::getFileNameExt(prefix));
	#ifdef OBJ_USE_OPENMP
	bool bSuccess(true);
	#pragma omp parallel for
	#endif
	for (int_t i = 0; i < (int_t)materials.size(); ++i) {
		const Material& mat = materials[i];
		// save material description
		out << "newmtl " << mat.name << "\n"
			<< "Ka 1.000000 1.000000 1.000000" << "\n"
			<< "Kd " << mat.Kd.r << " " << mat.Kd.g << " " << mat.Kd.b << "\n"
			<< "Ks 0.000000 0.000000 0.000000" << "\n"
			<< "Tr 1.000000" << "\n"
			<< "illum 1" << "\n"
			<< "Ns 1.000000" << "\n";
		// save material maps
		if (mat.diffuse_map.empty())
			continue;
		if (mat.diffuse_name.IsEmpty())
			const_cast<String&>(mat.diffuse_name) = name+"_"+mat.name+"_map_Kd."+(texLossless?"png":"jpg");
		out << "map_Kd " << mat.diffuse_name << "\n";
		const bool bRet(mat.diffuse_map.Save(pathName+mat.diffuse_name));
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
	const size_t numMaterials(materials.size());
	std::ifstream in(fileName.c_str());
	String keyword;
	while (in.good() && in >> keyword) {
		if (keyword == "newmtl") {
			in >> keyword;
			materials.push_back(Material(keyword));
		} else if (keyword == "Kd") {
			ASSERT(numMaterials < materials.size());
			Color c;
			in >> c.r >> c.g >> c.b;
			materials.back().Kd = c;
		} else if (keyword == "map_Kd") {
			ASSERT(numMaterials < materials.size());
			String& diffuse_name = materials.back().diffuse_name;
			in >> diffuse_name;
			diffuse_name = Util::getFilePath(fileName) + diffuse_name;
		}
	}
	return numMaterials < materials.size();
}
/*----------------------------------------------------------------*/



// S T R U C T S ///////////////////////////////////////////////////

bool ObjModel::Save(const String& fileName, unsigned precision, bool texLossless) const
{
	if (vertices.empty())
		return false;
	const String prefix(Util::getFileFullName(fileName));
	const String name(Util::getFileNameExt(prefix));

	if (!material_lib.Save(prefix, texLossless))
		return false;

	std::ofstream out((prefix + ".obj").c_str());
	if (!out.good())
		return false;

	out << "mtllib " << name << ".mtl" << "\n";

	out << std::fixed << std::setprecision(precision);
	for (size_t i = 0; i < vertices.size(); ++i) {
		out << "v "
			<< vertices[i][0] << " "
			<< vertices[i][1] << " "
			<< vertices[i][2] << "\n";
	}

	for (size_t i = 0; i < texcoords.size(); ++i) {
		out << "vt "
			<< texcoords[i][0] << " "
			<< texcoords[i][1] << "\n";
	}

	for (size_t i = 0; i < normals.size(); ++i) {
		out << "vn "
			<< normals[i][0] << " "
			<< normals[i][1] << " "
			<< normals[i][2] << "\n";
	}

	for (size_t i = 0; i < groups.size(); ++i) {
		out << "usemtl " << groups[i].material_name << "\n";
		for (size_t j = 0; j < groups[i].faces.size(); ++j) {
			Face const & face =  groups[i].faces[j];
			out << "f";
			for (size_t k = 0; k < 3; ++k) {
				out << " " << face.vertices[k]  + OBJ_INDEX_OFFSET
					<< "/" << face.texcoords[k]  + OBJ_INDEX_OFFSET
					<< "/" << face.normals[k]  + OBJ_INDEX_OFFSET;
			}
			out << "\n";
		}
	}
	return true;
}

bool ObjModel::Load(const String& fileName)
{
	ASSERT(vertices.empty() && groups.empty() && material_lib.materials.empty());
	std::ifstream fin(fileName.c_str());
	String line, keyword;
	std::istringstream in;
	while (fin.good()) {
		std::getline(fin, line);
		if (line.empty() || line[0u] == '#')
			continue;
		in.str(line);
		in >> keyword;
		if (keyword == "v") {
			Vertex v;
			in >> v[0] >> v[1] >> v[2];
			if (in.fail())
				return false;
			vertices.push_back(v);
		} else
		if (keyword == "vt") {
			TexCoord vt;
			in >> vt[0] >> vt[1];
			if (in.fail())
				return false;
			texcoords.push_back(vt);
		} else
		if (keyword == "vn") {
			Normal vn;
			in >> vn[0] >> vn[1] >> vn[2];
			if (in.fail())
				return false;
			normals.push_back(vn);
		} else
		if (keyword == "f") {
			Face f;
			memset(&f, 0xFF, sizeof(Face));
			for (size_t k = 0; k < 3; ++k) {
				in >> keyword;
				switch (sscanf(keyword, "%u/%u/%u", f.vertices+k, f.texcoords+k, f.normals+k)) {
				case 1:
					f.vertices[k] -= OBJ_INDEX_OFFSET;
					break;
				case 2:
					f.vertices[k] -= OBJ_INDEX_OFFSET;
					if (f.texcoords[k] != NO_ID)
						f.texcoords[k] -= OBJ_INDEX_OFFSET;
					if (f.normals[k] != NO_ID)
						f.normals[k] -= OBJ_INDEX_OFFSET;
					break;
				case 3:
					f.vertices[k] -= OBJ_INDEX_OFFSET;
					f.texcoords[k] -= OBJ_INDEX_OFFSET;
					f.normals[k] -= OBJ_INDEX_OFFSET;
					break;
				default:
					return false;
				}
			}
			if (in.fail())
				return false;
			groups.back().faces.push_back(f);
		} else
		if (keyword == "mtllib") {
			in >> keyword;
			if (!material_lib.Load(keyword))
				return false;
		} else
		if (keyword == "usemtl") {
			Group group;
			in >> group.material_name;
			if (in.fail())
				return false;
			groups.push_back(group);
		}
		in.clear();
	}
	return !vertices.empty();
}


ObjModel::Group& ObjModel::AddGroup(const String& material_name)
{
	groups.push_back(Group());
	Group& group = groups.back();
	group.material_name = material_name;
	if (!GetMaterial(material_name))
		material_lib.materials.push_back(MaterialLib::Material(material_name));
	return group;
}

ObjModel::MaterialLib::Material* ObjModel::GetMaterial(const String& name)
{
	MaterialLib::Materials::iterator it(std::find_if(material_lib.materials.begin(), material_lib.materials.end(), [&name](const MaterialLib::Material& mat) { return mat.name == name; }));
	if (it == material_lib.materials.end())
		return NULL;
	return &(*it);
}
/*----------------------------------------------------------------*/
