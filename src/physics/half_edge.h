/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: half_edge.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/20/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "math_utils.h"
#include <vector>

struct HalfEdge;
struct HalfEdgeFace;
class  HalfEdgeMesh;

struct HalfEdge
{
	unsigned vertex;

	HalfEdge* twin{ nullptr };
	HalfEdge* next{ nullptr };
	HalfEdge* prev{ nullptr };

	HalfEdgeFace* face;
};


struct HalfEdgeFace
{
	void clear();

	void refresh( HalfEdge* edge );

	HalfEdge* m_edge;
	vec3 m_normal;
	std::vector<unsigned> m_vertices;
};


class HalfEdgeMesh
{
public:

	~HalfEdgeMesh();
	void clear();

	const std::vector<vec3>&			vertices		() const;
	const std::vector<unsigned>&		indices			() const;
	const std::vector<unsigned>&		render_indices	() const;
	const std::vector<HalfEdgeFace*>&	faces			() const;


	void add_vertices	( const std::vector<vec3>& vertices );
	void add_face		( const unsigned p, const unsigned q, const unsigned r );
	void add_face		( HalfEdgeFace* face );

	const unsigned						render_mesh		() const;
	void set_render_mesh_id( const unsigned id );

	void link_twins();
	void merge_faces();
	void set_indices();
	mat3 compute_intertia_tensor() const;

public:
	vec3 hill_climbing( const vec3 dir );
	vec3 hill_climbing_bruteforce( const vec3 dir );



private:
	std::vector<vec3>			m_vertices;
	std::vector<unsigned>		m_render_indices;
	std::vector<unsigned>		m_indices;
	std::vector<HalfEdgeFace*>	m_faces;

	unsigned m_render_mesh;
};