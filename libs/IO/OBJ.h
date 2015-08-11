////////////////////////////////////////////////////////////////////
// OBJ.h
//
// Copyright 2007 cDc@seacave
// Distributed under the Boost Software License, Version 1.0
// (See http://www.boost.org/LICENSE_1_0.txt)

#ifndef __SEACAVE_OBJ_H__
#define __SEACAVE_OBJ_H__


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////


namespace SEACAVE {

// S T R U C T S ///////////////////////////////////////////////////

/*

OBJ model files parser.
(originally by Michael Waechter, modified by cDc@seacave)

The OBJ file format is a simple data-format that represents 3D geometry alone —
namely, the position of each vertex, the UV position of each texture coordinate
vertex, vertex normals, and the faces that make each polygon defined as a list of
vertices, and texture vertices. Vertices are stored in a counter-clockwise order
by default, making explicit declaration of face normals unnecessary.

*/

/*----------------------------------------------------------------*/

// Class representing a OBJ model.
class ObjModel {
public:
	// Class representing a material lib of an OBJ model.
	class MaterialLib {
	public:
		// Class representing a Lambertian material.
		class Material {
		private:
			Image8U3 diffuse_map;

		public:
			Material(const Image8U3& _diffuse_map);

			const Image8U3& get_diffuse_map() const { return diffuse_map; }
		};

	private:
		std::vector<Material> materials;
		std::vector<std::string> material_names;

	public:
		MaterialLib();

		void add_material(const std::string& name, Material material);
		inline size_t size() const { return materials.size(); }

		// Saves the material lib to an .mtl file and all textures of its materials with the given prefix.
		bool save(const std::string& prefix) const;
	};

public:
	struct Face {
		size_t vertices[3];
		size_t texcoords[3];
		size_t normals[3];
	};

	struct Group {
		std::string material_name;
		std::vector<Face> faces;
	};

	typedef std::vector<Vec3f> Vertices;
	typedef std::vector<Vec2f> TexCoords;
	typedef std::vector<Vec3f> Normals;
	typedef std::vector<Group> Groups;

private:
	Vertices vertices;
	TexCoords texcoords;
	Normals normals;
	Groups groups;
	MaterialLib material_lib;

public:
	// Saves the obj model to an .obj file, its material lib and the materials with the given prefix.
	bool save(const std::string& prefix) const;
	ObjModel();

	MaterialLib& get_material_lib() { return material_lib; }
	Vertices& get_vertices() { return vertices; }
	TexCoords& get_texcoords() { return texcoords; }
	Normals& get_normals() { return normals; }
	Groups& get_groups() { return groups; }

	static bool save(const ObjModel& model, const std::string& prefix);
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_OBJ_H__
