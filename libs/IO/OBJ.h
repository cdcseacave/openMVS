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

// OBJ model files parser.
// 
// The OBJ file format is a simple data-format that represents 3D geometry alone —
// namely, the position of each vertex, the UV position of each texture coordinate
// vertex, vertex normals, and the faces that make each polygon defined as a list of
// vertices, and texture vertices. Vertices are stored in a counter-clockwise order
// by default, making explicit declaration of face normals unnecessary.

class ObjModel {
public:
	typedef Pixel32F Color;

	// represents a material lib of an OBJ model
	struct MaterialLib {
		// represents a Lambertian material
		struct Material {
			Image8U3 diffuse_map;
			Color Kd;

			Material() : Kd(Color::WHITE) {}
			Material(const Image8U3& _diffuse_map, const Color& _Kd=Color::WHITE);
		};

		std::vector<Material> materials;
		std::vector<String> material_names;

		MaterialLib();

		void AddMaterial(const String& name, Material material);
		inline size_t GetSize() const { return materials.size(); }

		// Saves the material lib to a .mtl file and all textures of its materials with the given prefix
		bool Save(const String& prefix) const;
		// Loads the material lib from a .mtl file and all textures of its materials with the given prefix
		bool Load(const String& fileName);
	};

	struct Face {
		size_t vertices[3];
		size_t texcoords[3];
		size_t normals[3];
	};

	struct Group {
		String material_name;
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
	ObjModel();

	// Saves the obj model to an .obj file, its material lib and the materials with the given prefix
	bool Save(const String& prefix) const;
	// Loads the obj model from an .obj file, its material lib and the materials with the given prefix
	bool Load(const String& fileName);

	MaterialLib& get_material_lib() { return material_lib; }
	Vertices& get_vertices() { return vertices; }
	TexCoords& get_texcoords() { return texcoords; }
	Normals& get_normals() { return normals; }
	Groups& get_groups() { return groups; }
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_OBJ_H__
