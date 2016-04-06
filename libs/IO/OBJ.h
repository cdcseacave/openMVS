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

class IO_API ObjModel {
public:
	typedef Pixel32F Color;

	// represents a material lib of an OBJ model
	struct MaterialLib {
		// represents a Lambertian material
		struct Material {
			String name;
			String diffuse_name;
			Image8U3 diffuse_map;
			Color Kd;

			Material() : Kd(Color::WHITE) {}
			Material(const String& _name) : name(_name), Kd(Color::WHITE) {}
			Material(const Image8U3& _diffuse_map, const Color& _Kd=Color::WHITE);

			// Makes sure the image is loaded for the diffuse map
			bool LoadDiffuseMap();
		};

		typedef std::vector<Material> Materials;

		Materials materials;

		MaterialLib();

		// Saves the material lib to a .mtl file and all textures of its materials with the given prefix name
		bool Save(const String& prefix, bool texLossless=false) const;
		// Loads the material lib from a .mtl file and all textures of its materials with the given file name
		bool Load(const String& fileName);
	};

	typedef Point3f Vertex;
	typedef Point2f TexCoord;
	typedef Point3f Normal;

	typedef uint32_t Index;

	struct Face {
		Index vertices[3];
		Index texcoords[3];
		Index normals[3];
	};

	struct Group {
		String material_name;
		std::vector<Face> faces;
	};

	typedef std::vector<Vertex> Vertices;
	typedef std::vector<TexCoord> TexCoords;
	typedef std::vector<Normal> Normals;
	typedef std::vector<Group> Groups;

protected:
	Vertices vertices;
	TexCoords texcoords;
	Normals normals;
	Groups groups;
	MaterialLib material_lib;

public:
	ObjModel() {}

	// Saves the obj model to an .obj file, its material lib and the materials with the given file name
	bool Save(const String& fileName, unsigned precision=6, bool texLossless=false) const;
	// Loads the obj model from an .obj file, its material lib and the materials with the given file name
	bool Load(const String& fileName);

	// Creates a new group with the given material name
	Group& AddGroup(const String& material_name);
	// Retrieves a material from the library based on its name
	MaterialLib::Material* GetMaterial(const String& name);

	MaterialLib& get_material_lib() { return material_lib; }
	Vertices& get_vertices() { return vertices; }
	TexCoords& get_texcoords() { return texcoords; }
	Normals& get_normals() { return normals; }
	Groups& get_groups() { return groups; }
};
/*----------------------------------------------------------------*/

} // namespace SEACAVE

#endif // __SEACAVE_OBJ_H__
