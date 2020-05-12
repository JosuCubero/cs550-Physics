/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: test_half_edge.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/20/2020
----------------------------------------------------------------------------------------------------------*/
#include "half_edge.h"
#include "mesh.h"

#include <gtest/gtest.h>

TEST( half_edge, creating_a_face_links_edges_correctly )
{
	vec3 p = {  0.0f, 0.0f, 0.0f };
	vec3 q = {  1.0f, 2.0f, 1.0f };
	vec3 r = { -1.0f, 1.0f, 0.5f };

	std::vector<vec3> vertices;
	vertices.push_back( p );
	vertices.push_back( q );
	vertices.push_back( r );

	HalfEdgeMesh mesh;
	mesh.add_vertices( vertices );
	mesh.add_face( 0u, 1u, 2u );

	auto faces = mesh.faces();
	
	ASSERT_EQ( faces[0]->m_edge, faces[0]->m_edge->next->next->next );
}

TEST( half_edge, link_twins )
{
	vec3 p = {  0.0f, 0.0f, 0.0f };
	vec3 q = {  0.0f, 2.0f, 1.0f };
	vec3 r = { -1.0f, 1.0f, 0.5f };
	vec3 s = {  1.0f, 1.0f, 0.5f };

	std::vector<vec3> vertices;
	vertices.push_back( p );
	vertices.push_back( q );
	vertices.push_back( r );
	vertices.push_back( s );

	HalfEdgeMesh mesh;
	mesh.add_vertices( vertices );
	mesh.add_face( 0u, 1u, 2u );
	mesh.add_face( 1u, 0u, 3u );

	mesh.link_twins();

	auto faces = mesh.faces();

	ASSERT_EQ( faces[0]->m_edge->next->twin, faces[1]->m_edge->next );
}

TEST( half_edge, merge_coplanar_faces )
{
	vec3 p = vec3(  0.0f,  0.0f,  0.0f );
	vec3 q = vec3(  0.0f,  1.0f,  0.0f );
	vec3 r = vec3( -0.5f,  0.5f,  0.0f );
	vec3 s = vec3(  0.5f,  0.5f,  0.0f );

	std::vector<vec3> vertices;
	vertices.push_back( p );
	vertices.push_back( q );
	vertices.push_back( r );
	vertices.push_back( s );

	HalfEdgeMesh mesh;
	mesh.add_vertices( vertices );
	mesh.add_face( 0u, 1u, 2u );
	mesh.add_face( 1u, 0u, 3u );

	mesh.link_twins();
	mesh.merge_faces();

	ASSERT_EQ( mesh.faces().size(), 1u );
}

TEST( half_edge, a_cube_has_6_faces )
{
	// load OBJs
	Mesh cube_mesh = load_obj( "../resources/meshes/cube.obj" );

	HalfEdgeMesh cube;
	cube.add_vertices( cube_mesh.vertices );
	for ( unsigned i = 0; i < cube_mesh.indices.size(); i++ )
		cube.add_face(	cube_mesh.indices[i].x, 
						cube_mesh.indices[i].y,
						cube_mesh.indices[i].z );

	cube.link_twins();
	cube.merge_faces();

	ASSERT_EQ( cube.faces().size(), 6u );
}

TEST( half_edge, hill_climbing )
{

	// load OBJs
	Mesh cube_mesh = load_obj("../resources/meshes/cube.obj");

	HalfEdgeMesh cube;
	cube.add_vertices(cube_mesh.vertices);
	for (unsigned i = 0; i < cube_mesh.indices.size(); i++)
		cube.add_face(cube_mesh.indices[i].x,
			cube_mesh.indices[i].y,
			cube_mesh.indices[i].z);

	cube.link_twins();
	cube.merge_faces();

	vec3 vertex = cube.hill_climbing( normalize( vec3( 1.0f, 1.0f, 1.0f ) ) );
	ASSERT_EQ( vertex, vec3( 0.5f, 0.5f, 0.5f ) );
}