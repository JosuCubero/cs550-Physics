/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: scene.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 02/22/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "camera.h"
#include "rigid_body.h"

#include "math_utils.h"

#include <string>

class Scene
{
public:		// LOADING - UNLOADING
	Scene() = default;

	void clear();

	Scene( const char* filename );
	void load_scene( const char* filename );

private:	// PARSER
	void		read_line	( std::string& line );
	RigidBody	read_body	( std::string& data );
	vec3		read_vector	( std::string& data );
	float		read_float	( std::string& data, const char prev_char, const char post_char );
	float		read_float	( std::string& data );

private:	// SHAPE SPECIFICS
	void add_cube		 ( std::string& data );
	void add_cylinder	 ( std::string& data );
	void add_icosahedron ( std::string& data );
	void add_octohedron	 ( std::string& data );
	void add_sphere		 ( std::string& data );
};