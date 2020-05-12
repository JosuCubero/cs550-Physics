/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: mesh.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/24/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "math_utils.h"
#include <vector>

struct Mesh
{
	std::vector<vec3> vertices;
	std::vector<ivec3> indices;
};

Mesh load_obj( const char* file_path );
