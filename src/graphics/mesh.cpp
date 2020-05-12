/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: mesh.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/24/2020
----------------------------------------------------------------------------------------------------------*/
#include "mesh.h"

#include <string>
#include <fstream>
#include <iostream>

/**
* @brief create a mesh from an obj file
* @param file_path	.obj file
* @return mesh
*/
Mesh load_obj( const char* file_path )
{
	Mesh mesh;

	// mesh file
	std::ifstream file( file_path );

	if ( !file.is_open() )
	{
		std::cout << "Couldn't open the file " << file_path << std::endl;
		std::abort();
	}

	while ( !file.eof() )
	{
		std::string line;
		file >> line;

		// vertex
		if ( line == "v" )
		{
			vec3 vertex;

			std::string val;
			file >> val;
			vertex.x = static_cast<float>( std::atof( val.c_str() ) );
			file >> val;
			vertex.y = static_cast<float>( std::atof( val.c_str() ) );
			file >> val;
			vertex.z = static_cast<float>( std::atof( val.c_str() ) );

			mesh.vertices.push_back( vertex );
		}
		// face index
		else if ( line == "f" )
		{
			ivec3 index;

			std::string val;
			file >> val;
			index.x = static_cast<float>( std::atof( val.c_str() ) );
			file >> val;
			index.y = static_cast<float>( std::atof( val.c_str() ) );
			file >> val;
			index.z = static_cast<float>( std::atof( val.c_str() ) );

			mesh.indices.push_back( index - glm::one<ivec3>() );
		}
		// extra
		else
		{
			std::string useless;
			std::getline( file, useless );
		}
	}

	file.close();

	return mesh;
}
