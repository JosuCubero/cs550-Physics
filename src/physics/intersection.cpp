/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: intersection.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/27/2020
----------------------------------------------------------------------------------------------------------*/
#include "intersection.h"

/**
* @brief compute the collision between a ray and a polyhedra
* @param ray
* @param polyhedra
* @return contact information
*/
Contact intersection_ray_polyhedra( const Ray ray, const HalfEdgeMesh* polyhedra )
{
	auto faces = polyhedra->faces();
	auto& vertices = polyhedra->vertices();

	Contact contact;
	contact.time = -1.0f;

	// check raycast for each face
	for ( HalfEdgeFace* face : faces )
	{
		Contact temp = intersection_ray_polygon( ray, vertices, face->m_vertices );

		if ( temp.time != -1.0f && ( temp.time < contact.time || contact.time == -1.0f ) )
			contact = temp;
	}

	return contact;
}

/**
* @brief compute the collision between a ray and a polygon
* @param ray
* @param polygon
* @return contact information
*/
Contact intersection_ray_polygon( const Ray ray, const std::vector<vec3>& vertices, const std::vector<unsigned>& indices )
{
	Contact contact;
	contact.time = -1.0f;

	// check raycast for each triangle
	for ( unsigned i = 1u; i < indices.size() - 1; i++ )
	{
		Contact temp = intersection_ray_triangle( ray, vertices[indices[0u]], vertices[indices[i]], vertices[indices[i + 1u]] );

		if ( temp.time != -1.0f && ( temp.time < contact.time || contact.time == -1.0f ) )
			contact = temp;
	}

	return contact;
}

/**
* @brief compute the collision between a ray and a triangle
* @param ray
* @param a		point in the triangle
* @param b		point in the triangle
* @param c		point in the triangle
* @return contact information
*/
Contact intersection_ray_triangle( const Ray ray, const vec3 a, const vec3 b, const vec3 c )
{
	Contact contact;

	vec3 ab = b - a;
	vec3 ac = c - a;

	// normal vector of the triangle
	vec3 normal = contact.normal = cross( ab, ac );

	float div = dot( normal, ray.direction );

	// check
	if ( glm::abs( div ) < 0.01f )
	{
		contact.time = -1.0f;
		return contact;
	}

	// time of collision
	float time = contact.time = ( dot( normal, a ) - dot( normal, ray.origin ) ) / div;

	// check
	if ( time < 0.0f )
	{
		contact.time = -1.0f;
		return contact;
	}

	vec3 point = contact.position = ray.origin + time * ray.direction;
	vec3 coords;

	// out of bounds
	if ( get_barycentric_coordinates( a, b, c, point, &coords ) == false )
	{
		contact.time = -1.0f;
		return contact;
	}

	return contact;
}

/**
* @brief get barycentric coordinates of a point in a triangle
* @param a		point in the triangle
* @param b		point in the triangle
* @param c		point in the triangle
* @param point
* @param result	coordinate for each axis
* @return inside
*/
bool get_barycentric_coordinates( const vec3 a, const vec3 b, const vec3 c, const vec3 &point, vec3 *result )
{
	vec3 v0 = b - a;
	vec3 v1 = c - a;
	vec3 v2 = point - a;

	float v0v0 = dot( v0, v0 );
	float v0v1 = dot( v0, v1 );
	float v1v1 = dot( v1, v1 );
	float uv0 = dot( v2, v0 );
	float uv1 = dot( v2, v1 );

	float div = v0v0 * v1v1 - v0v1 * v0v1;

	// check
	if ( div == 0 )
		return false;

	// set result
	result->y = ( v1v1 * uv0 - v0v1 * uv1 ) / div;
	result->z = ( v0v0 * uv1 - v0v1 * uv0 ) / div;
	result->x = 1 - result->y - result->z;

	// check boundaries
	if ( result->x < 0 || result->y < 0 || result->z < 0 )
		return false;

	return true;
}
