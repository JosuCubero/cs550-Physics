/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: intersection.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/27/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "half_edge.h"

#include "math_utils.h"
#include <vector>

struct Contact
{
	vec3 position;
	vec3 normal;
	float time;
};

struct Ray
{
	vec3 origin;
	vec3 direction;
};

Contact intersection_ray_polyhedra	( const Ray ray, const HalfEdgeMesh* polyhedra );
Contact	intersection_ray_polygon	( const Ray ray, const std::vector<vec3>& vertices, const std::vector<unsigned>& indices );
Contact intersection_ray_triangle	( const Ray ray, const vec3 p, const vec3 q, const vec3 r );
bool get_barycentric_coordinates	( const vec3 a, const vec3 b, const vec3 c, const vec3 &point, vec3 *result );
