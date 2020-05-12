/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: collision.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 02/10/2020
----------------------------------------------------------------------------------------------------------*/
#include "collision.h"

#include "graphics.h"

std::pair<vec3, vec3> closest_points_segment( const vec3& a0, const vec3& a1, const vec3& b0, const vec3& b1 );

/**
* @brief check if to bodies collide
* @param body_A
* @param body_B
* @return contact_data	contact manifold of the collision
* @return the bodies are colliding
*/
bool overlap_sat( RigidBody& body_A, RigidBody& body_B, ContactManifold& contact_data )
{
	const float epsilon = 0.005f;

	// all faces of polygon A
	ContactFace contact_A = has_separating_axis_face( body_A, body_B );
	if ( contact_A.separation > 0.0f )
		return false;

	// all faces of polygon B
	ContactFace contact_B = has_separating_axis_face( body_B, body_A );
	if ( contact_B.separation > 0.0f )
		return false;

	// all edges of A and B
	ContactEdge contact_edge = has_separating_axis_edge(body_A, body_B);
	if ( contact_edge.separation > 0.0f )
		return false;

	if ( contact_A.separation == -std::numeric_limits<float>::max() && 
		 contact_B.separation == -std::numeric_limits<float>::max() &&
		 contact_edge.separation == -std::numeric_limits<float>::max() )
		return false;


	// at this point there is no separation axis so the two bodies are colliding
	int collision_case = 0;
	if ( contact_A.separation > contact_B.separation - epsilon )
	{
		if ( contact_A.separation < contact_edge.separation - epsilon )
			collision_case = 2;
	}
	else
	{
		if ( contact_B.separation < contact_edge.separation - epsilon )
			collision_case = 2;
		else
			collision_case = 1;
	}

	switch ( collision_case )
	{
	case 0:
		contact_data = get_contact_manifold( body_A, body_B, contact_A );
		break;
	case 1:
		contact_data = get_contact_manifold( body_B, body_A, contact_B );
		break;
	case 2:
		contact_data = get_contact_manifold( body_A, body_B, contact_edge );
		break;
	}

	return true;
}

/**
* @brief check if there is any separating axis between faces and vertices
* @param body_A		body to check faces
* @param body_B		body to check vertices
* @return there is a separating axis
*/
ContactFace has_separating_axis_face( const RigidBody& body_A, const RigidBody& body_B )
{
	ContactFace contact;

	// transformation matrices of the bodies
	const mat4 trs_A = body_A.model();
	const mat4 trs_B = body_B.model();

	// transformation from body A space to body B space and inverse
	const mat4 trs		= inverse( trs_B ) * trs_A;
	const mat4 inv_trs	= inverse( trs );

	// half edge meshes of the bodies
	auto mesh_A = body_A.mesh;
	auto mesh_B = body_B.mesh;

	// for every face in body A
	for ( unsigned i = 0u; i < mesh_A->faces().size(); i++ )
	{
		auto& face_A = mesh_A->faces()[i];

		// normal of face A in B space
		vec3 dir = vec3( trs * vec4( -face_A->m_normal, 0.0f ) );

		// get the support point of B given the direction
		vec3 support_B = mesh_B->hill_climbing( dir );

		// transform the obtained point to A space
		vec3 support = vec3( inv_trs * vec4( support_B, 1.0f ) );

		// compute the distance from the point to the face
		vec3 v = support - mesh_A->vertices()[face_A->m_vertices[0]];
		float dist = dot( v, face_A->m_normal );

		if ( dist > contact.separation )
		{
			contact.separation = dist;
			contact.face_id = i;

			// if the distance is bigger than 0 there is a separating axis
			if ( dist > 0.0f )
				return contact;
		}
	} // O(n)
	
	contact.separation = contact.separation;

	return contact;
}

/**
* @brief check if there is any separating axis between edges
* @param body_A		body to check faces
* @param body_B		body to check vertices
* @return there is a separating axis
*/
ContactEdge has_separating_axis_edge( const RigidBody& body_A, const RigidBody& body_B )
{
	ContactEdge contact;

	// half edge meshes of the bodies
	auto mesh_A = body_A.mesh;
	auto mesh_B = body_B.mesh;

	// for every face in body A
	for ( auto face_A : mesh_A->faces() )
	{
		// for every face in body B
		for ( auto face_B : mesh_B->faces() )
		{
			// for every edge in face A
			auto edge_A = face_A->m_edge;
			do
			{
				// for every edge in face B
				auto edge_B = face_B->m_edge;
				do
				{
					if ( create_minkowski_face( edge_A, edge_B, body_A, body_B ) )
					{
						float dist = edge_distance(edge_A, edge_B, body_A, body_B);
						if ( dist > contact.separation )
						{
							contact.separation = dist;
							contact.edge_A = edge_A;
							contact.edge_B = edge_B;

							if ( dist > 0.0f )
								return contact;
						}
					}
					
					edge_B = edge_B->next;
				} while ( edge_B != face_B->m_edge );

				edge_A = edge_A->next;
			} while ( edge_A != face_A->m_edge );
		}
	} // O(n*m * p*q)

	return contact;
}

/**
* @brief build a minkowski face given two edges
* @param edge_A
* @param edge_B
* @param trs_A	transformation matrix from body A to world coordinates
* @param trs_B	transformation matrix from body B to world coordinates
* @return the edges create a minkowski face
*/
bool create_minkowski_face( const HalfEdge* edge_A, const HalfEdge* edge_B, const RigidBody& body_A, const RigidBody& body_B )
{
	// transformation matrices of the bodies
	const mat4 trs_A = body_A.model();
	const mat4 trs_B = body_B.model();

	vec3 a = edge_A->face->m_normal;
	vec3 b = edge_A->twin->face->m_normal;
	vec3 c = edge_B->face->m_normal;
	vec3 d = edge_B->twin->face->m_normal;

	a = vec3( trs_A * vec4( a, 0.0f ) );
	b = vec3( trs_A * vec4( b, 0.0f ) );
	c = vec3( trs_B * vec4( c, 0.0f ) );
	d = vec3( trs_B * vec4( d, 0.0f ) );

	return is_minkowski_face( a, b, -c, -d );
}

/**
* @brief	check if the given faces create a minkowski difference checking the intersection
			in the gaussian map computing the triple product
* @param a	normal of face in polyhedra A
* @param b	normal of face in polyhedra A
* @param c	normal of face in polyhedra B
* @param d	normal of face in polyhedra B
* @return if the faces form a minkowski face
*/
bool is_minkowski_face( const vec3 a, const vec3 b, const vec3 c, const vec3 d )
{
	vec3 bxa = cross( b, a ); // cross product of normals (edge dir, possible optimization)
	vec3 dxc = cross( d, c ); // same here 

	float cba = dot( bxa, c );
	float dba = dot( bxa, d );
	float adc = dot( dxc, a );
	float bdc = dot( dxc, b );

	return cba * dba < 0.0f && adc * bdc < 0.0f && cba * bdc > 0.0f;
}

/**
* @brief return the distance between two edges
* @param edge_A
* @param edge_B
* @param trs_A	transformation matrix from body A to world coordinates
* @param trs_B	transformation matrix from body B to world coordinates
* @return distance between edges
*/
float edge_distance( const HalfEdge* edge_A, const HalfEdge* edge_B, const RigidBody& body_A, const RigidBody& body_B )
{
	// transformation matrices of the bodies
	const mat4 trs_A = body_A.model();
	const mat4 trs_B = body_B.model();

	// vertices of the meshes
	auto vertices_A = body_A.mesh->vertices();
	auto vertices_B = body_B.mesh->vertices();

	vec3 point_A = vec3( trs_A * vec4( vertices_A[edge_A->vertex], 1.0f ) );
	vec3 point_B = vec3( trs_B * vec4( vertices_B[edge_B->vertex], 1.0f ) );
	vec3 dir_A = normalize( point_A - vec3( trs_A * vec4( vertices_A[edge_A->prev->vertex], 1.0f ) ) );
	vec3 dir_B = normalize( point_B - vec3( trs_B * vec4( vertices_B[edge_B->prev->vertex], 1.0f ) ) );

	// parallel edges
	if ( std::abs( dot( dir_A, dir_B ) ) == 1.0f )
		return -std::numeric_limits<float>::max();

	vec3 normal = normalize( cross( dir_A, dir_B ) );

	if ( dot( normal, point_A - body_A.position ) < 0.0f )
		normal = -normal;

	return dot( normal, point_B - point_A );
}

#include <iostream>
/**
* @brief get the contact points from the overlapped face
* @param body_A		body with reference face
* @param body_B		body with incident face
* @param reference_face_index
* @return contact manifold
*/
ContactManifold get_contact_manifold( RigidBody& body_A, RigidBody& body_B, ContactFace& incident_contact )
{
	// relevant faces
	HalfEdgeFace* reference = body_A.mesh->faces()[incident_contact.face_id];
	HalfEdgeFace* incident;

	// transformation matrices
	const mat4 trs_A = body_A.model();
	const mat4 trs_B = body_B.model();
	const mat4 inv_trs_B = inverse( trs_B );

	// antinormal vecetor
	vec3 antinormal = normalize( vec3( trs_A * vec4( -reference->m_normal, 0.0f ) ) );

	// get the most opposite face from body B
	float max_dot = -1.0f;
	for ( HalfEdgeFace* face : body_B.mesh->faces() )
	{
		vec3 normal = vec3( trs_B * vec4( face->m_normal, 0.0f ) );
		float dot = glm::dot( antinormal, normal );
		if ( dot > max_dot )
		{
			max_dot = dot;
			incident = face;
		}
	} // O(n)

	// copy the vertices of the face in world coordinates
	std::vector<vec3> face_points;
	std::vector<vec3> contact_points;

	for ( unsigned i = 0u; i < incident->m_vertices.size(); i++ )
		face_points.push_back( vec3( trs_B * vec4( body_B.mesh->vertices()[incident->m_vertices[i]], 1.0f ) ) );


	HalfEdge* edge_it = reference->m_edge;
	do
	{
		HalfEdgeFace* clipping_plane = edge_it->twin->face;
		// normal and point in world coordinates
		vec3 normal = vec3( trs_A * vec4( clipping_plane->m_normal, 0.0f ) );
		vec3 point	= vec3( trs_A * vec4( body_A.mesh->vertices()[clipping_plane->m_vertices[0u]], 1.0f ) );

		// clip the vertices with the adjacent face
		unsigned size = static_cast<unsigned>( face_points.size() );
		for ( unsigned i = 0; i < size; i++ )
		{
			// intersect line against plane
			float t1 = distance_point_plane( face_points[i], normal, point );
			float t2 = distance_point_plane( face_points[(i + 1u) % size], normal, point );

			// both points out
			if ( t1 > 0.0f && t2 > 0.0f )
				continue;
			
			// first point in, second point out
			if ( t2 > 0.0f )
			{
				float t = -t1 / ( -t1 + t2 );
				contact_points.push_back( ( face_points[( i + 1u ) % size] - face_points[i] ) * t + face_points[i] );
			}
			else
			{
				// first point out, second point in
				if ( t1 > 0.0f )
				{
					float t = t1 / (t1 - t2);
					contact_points.push_back( ( face_points[( i + 1u ) % size] - face_points[i] ) * t + face_points[i] );
				}
				// second point in
				contact_points.push_back( face_points[( i + 1u ) % size] );
			}
		}

		// save clipped points		
		face_points = contact_points;
		contact_points.clear();

		edge_it = edge_it->next;
	} while ( edge_it != reference->m_edge );

	// contact data
	assert( glm::length2( reference->m_normal ) > 0.0f );
	ContactManifold contact;
	contact.normal = vec3( trs_A * vec4( reference->m_normal, 0.0f ) );

	// ignore points outside the reference face
	for ( auto point : face_points )
	{
		vec3 face_normal = contact.normal;
		float penetration = distance_point_plane( point, face_normal, vec3( trs_A * vec4( body_A.mesh->vertices()[reference->m_vertices[0u]], 1.0f ) ) );
		if ( penetration <= 0.0f )
		{
			vec3 point_A = point - penetration * face_normal;
			contact.points.push_back( ContactPoint{ point_A, point, -penetration } );
		}
	}

	contact.body_A = &body_A;
	contact.body_B = &body_B;

	return contact;
}


/**
* @brief get the contact points from the overlapping edges
* @param edge_A		edge of body A
* @param edge_B		edge of body B
* @return contact manifold
*/
ContactManifold get_contact_manifold( RigidBody& body_A, RigidBody& body_B, const ContactEdge& contact_info )
{
	const HalfEdge* edge_A = contact_info.edge_A;
	const HalfEdge* edge_B = contact_info.edge_B;

	ContactManifold contact;

	if ( edge_A == nullptr )
		return contact;

	// transformation matrices
	const mat4 trs_A = body_A.model();
	const mat4 trs_B = body_B.model();

	const auto& vertices_A = body_A.mesh->vertices();
	const auto& vertices_B = body_B.mesh->vertices();

	// segment points
	const vec3 a0 = vec3( trs_A * vec4( vertices_A[edge_A->prev->vertex], 1.0f ) );
	const vec3 a1 = vec3( trs_A * vec4( vertices_A[edge_A->vertex], 1.0f ) );
	const vec3 b0 = vec3( trs_B * vec4( vertices_B[edge_B->prev->vertex], 1.0f ) );
	const vec3 b1 = vec3( trs_B * vec4( vertices_B[edge_B->vertex], 1.0f ) );

	// contact points
	const auto points = closest_points_segment( a0, a1, b0, b1 );
	

	contact.normal = normalize( points.first - body_A.position );

	assert( glm::length2( contact.normal ) > 0.0f );

	const float dist = -contact_info.separation;


	contact.points.push_back( { points.first, points.second, dist } );
	contact.body_A = &body_A;
	contact.body_B = &body_B;

	return contact;
}

/**
* @brief compute the distance from a point to a plane
* @param point
* @param plane_normal
* @param plane_point
* @return distance
*/
float distance_point_plane( const vec3& point, const vec3& plane_normal, const vec3& plane_point )
{
	return dot( plane_normal, point - plane_point );
}

/**
* @brief closest points between two segments
*/
std::pair<vec3, vec3> closest_points_segment( const vec3& a0, const vec3& a1, const vec3& b0, const vec3& b1 )
{
	const float epsilon = 0.00001f;

	const vec3 d1 = a1 - a0;
	const vec3 d2 = b1 - b0;
	const vec3 r = a0 - b0;

	const float a = length2( d1 );
	const float e = length2( d2 );

	if ( a <= epsilon && e <= epsilon )
		return { a0, b0 };

	const float f = dot( d2, r );

	float s, t;
	if ( a <= epsilon )
	{
		s = 0.0f;
		t = clamp( f / e, 0.0f, 1.0f );
	}
	else
	{
		const float c = dot( d1, r );

		if ( e <= epsilon )
		{
			t = 0.0f;
			s = clamp( -c / a, 0.0f, 1.0f );
		}
		else
		{
			const float b = dot( d1, d2 );
			const float denom = a * e - b * b;

			if ( denom != 0.0f )
				s = clamp( ( b * f - c * e ) / denom, 0.0f, 1.0f );
			else
				s = 0.0f;

			t = ( b * s + f ) / e;

			if ( t < 0.0f )
			{
				t = 0.0f;
				s = clamp( -c / a, 0.0f, 1.0f );
			}
			else if ( t > 1.0f )
			{
				t = 1.0f;
				s = clamp( ( b - c ) / a, 0.0f, 1.0f );
			}
		}
	}
	return { a0 + d1 * s, b0 + d2 * t };
}