/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: half_edge.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/20/2020
----------------------------------------------------------------------------------------------------------*/
#include "half_edge.h"


/*		HALF EDGE FACE		*/

/**
* @brief delete memory of the face
*/
void HalfEdgeFace::clear()
{
	auto it_edge = m_edge->next;

	while ( it_edge != m_edge )
	{
		auto remove_edge = it_edge;
		it_edge = it_edge->next;
		delete remove_edge;
	}

	delete m_edge;
}

/**
* @brief refresh face data
* @param edge	edge of the face
*/
void HalfEdgeFace::refresh( HalfEdge* edge )
{
	m_vertices.clear();

	// restore normal and edge pointer
	m_normal = edge->face->m_normal;
	m_edge = edge;

	auto it_edge = edge;

	// O(m)
	do
	{
		// link face with edges
		it_edge->face = this;

		it_edge = it_edge->next;
	} while ( it_edge != edge );
}






/*		HALF EDGE MESH		*/

/**
* @brief mesh destructor
*/
HalfEdgeMesh::~HalfEdgeMesh()
{
	clear();
}

/**
* @brief delete memory of the mesh
*/
void HalfEdgeMesh::clear()
{
	for ( auto face : m_faces )
	{
		face->clear();
		delete face;
	}

	m_faces.clear();
}






/**
* @brief return array of faces
* @return array of faces
*/
const std::vector<HalfEdgeFace*>& HalfEdgeMesh::faces() const
{
	return m_faces;
}

/**
* @brief get vertices
* @return vertices
*/
const std::vector<vec3>& HalfEdgeMesh::vertices() const
{
	return m_vertices;
}

/**
* @brief get vertex indices
* @return indices
*/
const std::vector<unsigned>& HalfEdgeMesh::indices() const
{
	return m_indices;
}

/**
* @brief get vertex indices for rendering
* @return indices
*/
const std::vector<unsigned>& HalfEdgeMesh::render_indices() const
{
	return m_render_indices;
}

/**
* @brief get index to the renderable mesh
* @return index
*/
const unsigned HalfEdgeMesh::render_mesh() const
{
	return m_render_mesh;
}

/**
* @brief set the renderable mesh index
* @param id
*/
void HalfEdgeMesh::set_render_mesh_id( const unsigned id )
{
	m_render_mesh = id;
}





/**
* @brief copy the set of vertices to the half edge mesh
* @param vertices
*/
void HalfEdgeMesh::add_vertices( const std::vector<vec3>& vertices )
{
	// add the vertices to the end of the vertices vector
	std::copy( vertices.begin(), vertices.end(), std::back_inserter( m_vertices ) );
}

/**
* @brief add a face to the mesh given 3 vertices
* @param p
* @param q
* @param r
*/
void HalfEdgeMesh::add_face( const unsigned p, const unsigned q, const unsigned r )
{
	// allocate memory for the edges
	HalfEdge* edge_1 = new HalfEdge;
	HalfEdge* edge_2 = new HalfEdge;
	HalfEdge* edge_3 = new HalfEdge;

	// vertex value
	edge_1->vertex = p;
	edge_2->vertex = q;
	edge_3->vertex = r;

	// set next edges
	edge_1->next = edge_2;
	edge_2->next = edge_3;
	edge_3->next = edge_1;

	// set previous edges
	edge_1->prev = edge_3;
	edge_2->prev = edge_1;
	edge_3->prev = edge_2;


	// set face
	HalfEdgeFace* face = new HalfEdgeFace;

	edge_1->face = face;
	edge_2->face = face;
	edge_3->face = face;

	// link face and edge
	face->m_edge = edge_1;

	// compute face normal
	face->m_normal = normalize( cross(	m_vertices[edge_3->vertex] - m_vertices[edge_2->vertex],
										m_vertices[edge_1->vertex] - m_vertices[edge_2->vertex] ) );

	face->m_vertices.push_back( edge_1->vertex );
	face->m_vertices.push_back( edge_2->vertex );
	face->m_vertices.push_back( edge_3->vertex );

	add_face( face );
}

/**
* @brief add a face to the mesh
* @param face
*/
void HalfEdgeMesh::add_face( HalfEdgeFace* face )
{
	m_faces.push_back( face );
}





/**
* @brief link the twin edges
*/
void HalfEdgeMesh::link_twins()
{
	// loop through all edges in faces	O(n^2 * m^2)
	for ( auto face_1 = m_faces.begin(); face_1 != m_faces.end() - 1u; face_1++ )
	{
		for ( auto face_2 = face_1 + 1u; face_2 != m_faces.end(); face_2++ )
		{
			// loop through edges in face 1
			auto edge_1 = ( *face_1 )->m_edge;

			do
			{
				// loop through edges in face 2
				auto edge_2 = ( *face_2 )->m_edge;

				do
				{
					// twin edge found
					if ( edge_1->vertex == edge_2->vertex && edge_1->prev->vertex == edge_2->next->vertex )
					{
						edge_1->twin = edge_2->next;
						edge_2->next->twin = edge_1;
					}

					edge_2 = edge_2->next;	// it++
				} while ( edge_2 != ( *face_2 )->m_edge );

				edge_1 = edge_1->next;	// it++
			} while ( edge_1 != ( *face_1 )->m_edge );
		}
	}
}

/**
* @brief merge coplanar faces
*/
void HalfEdgeMesh::merge_faces()
{
	// O(n*m)
	for ( unsigned i = 0u; i < m_faces.size(); i++ )
	{
		auto face = m_faces[i];

		// skip epty face
		if ( face->m_edge == nullptr )
			continue;

		auto edge = face->m_edge;

		// for every edge O(m)
		do
		{
			auto twin = edge->twin;

			// edge and twin faces share normal -> merge faces
			if ( twin != nullptr && edge->face->m_normal == twin->face->m_normal )
			{
				// both edges belong to the same face
				if ( edge->next == twin )
				{
					// link adjaject edges
					edge->prev->next = twin->next;
					twin->next->prev = edge->prev;

					// create a new face
					HalfEdgeFace* new_face = new HalfEdgeFace;

					// refresh new face data and add it to the faces
					new_face->refresh( edge->prev );
					m_faces.push_back( new_face );

					// mark face as deletable
					edge->face->m_edge = nullptr;

					// delete memory of the edges
					delete edge;
					delete twin;

					break;
				}

				// link edges
				edge->prev->next = twin->next;
				twin->next->prev = edge->prev;

				edge->next->prev = twin->prev;
				twin->prev->next = edge->next;


				// create new face
				HalfEdgeFace* new_face = new HalfEdgeFace;

				// refresh new face data and add it to the faces
				new_face->refresh( edge->prev );
				m_faces.push_back( new_face );
				

				// mark old faces as deletable
				edge->face->m_edge = nullptr;
				twin->face->m_edge = nullptr;

				// delete memory of the edges
				delete edge;
				delete twin;

				break;
			}

			edge = edge->next;

		} while ( edge != face->m_edge ); 
	}

	// remove empty faces O(n)
	for ( auto face_it = m_faces.begin(); face_it != m_faces.end(); )
	{
		if ( ( *face_it )->m_edge == nullptr )
		{
			delete ( *face_it );
			face_it = m_faces.erase( face_it );
			continue;
		}

		face_it++;
	}
}

/**
* @brief set the indices of each face
*/
void HalfEdgeMesh::set_indices()
{
	// O(n*m)
	for ( auto& face : m_faces )
	{
		face->m_vertices.clear();

		// O(m)
		auto edge = face->m_edge;
		do 
		{
			// add face vertex index and edge
			face->m_vertices.push_back( edge->vertex );
			m_indices.push_back( edge->vertex );

			// set rendering indeces
			m_render_indices.push_back( edge->vertex );
			m_render_indices.push_back( edge->next->vertex );


			edge = edge->next;
		} while ( edge != face->m_edge );
	}
}

/**
* @brief compute the intertia tensor matrix of the mesh
* @return intertia tensor matrix
*/
mat3 HalfEdgeMesh::compute_intertia_tensor() const
{
	// scalar multipliers of the polynomial
	const float mult[10] = { 1.0f/6.0f,
							 1.0f/24.0f,  1.0f/24.0f,  1.0f/24.0f,
							 1.0f/60.0f,  1.0f/60.0f,  1.0f/60.0f,
							 1.0f/120.0f, 1.0f/120.0f, 1.0f/120.0f };

	// in the order of      1,    x,    y,    z,   x^2,  y^2,  z^2,  x*y,  y*z,  z*x
	float integral[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	// for every face in the mesh O(n*(m-2)) 
	for ( unsigned i = 0u; i < m_faces.size(); i++ )
	{
		// current face
		const auto& face = m_faces[i];

		// loop from 1 to n-1 O(m-2)
		for ( unsigned j = 1u; j < face->m_vertices.size() - 1u; j++ )
		{
			// get triangle of the face
			unsigned i0 = face->m_vertices[0], i1 = face->m_vertices[j], i2 = face->m_vertices[j + 1u];
			vec3 a = m_vertices[i0];
			vec3 b = m_vertices[i1];
			vec3 c = m_vertices[i2];

			// get direction vectors of the face
			vec3 v = b - a;
			vec3 w = c - a;

			// get normal of the face
			//vec3 d = cross( v, w );
			vec3 d = face->m_normal;

			vec3 g0( 0.0f ), g1( 0.0f ), g2( 0.0f );
			vec3 f1( 0.0f ), f2( 0.0f ), f3( 0.0f );

			auto subexpression = [&]( const vec3 w0, const vec3 w1, const vec3 w2 )
			{
				vec3 temp0 = w0 + w1;
				vec3 temp1 = w0 * w0;
				vec3 temp2 = temp1 + w1 * temp0;

				f1 = temp0 + w2;
				f2 = temp2 + w2 * f1;
				f3 = w0 * temp1 + w1 * temp2 + w2 * f2;

				g0 = f2 + w0 * ( f1 + w0 );
				g1 = f2 + w1 * ( f1 + w1 );
				g2 = f2 + w2 * ( f1 + w2 );
			};

			subexpression( a, b, c );

			integral[0] += d.x * f1.x;
			
			integral[1] += d.x * f2.x;
			integral[2] += d.y * f2.y;
			integral[3] += d.z * f2.z;

			integral[4] += d.x * f3.x;
			integral[5] += d.y * f3.y;
			integral[6] += d.z * f3.z;

			integral[7] += d.x * ( a.y * g0.x + b.y * g1.x + c.y * g2.x );
			integral[8] += d.y * ( a.z * g0.y + b.z * g1.y + c.z * g2.y );
			integral[9] += d.z * ( a.x * g0.z + b.x * g1.z + c.x * g2.z );
		}
	}

	// apply polynomial multipliers
	for ( unsigned i = 0u; i < 10u; i++ )
		integral[i] *= mult[i];

	// get mass of the mesh (assuming density 1)
	float mass = integral[0];

	// compute center of mass
	vec3 cm( integral[1] / mass, integral[2] / mass, integral[3] / mass );

	// compute inertia tensor values
	const float xx = integral[5] + integral[6] - mass * ( cm.y*cm.y + cm.z*cm.z );
	const float yy = integral[4] + integral[6] - mass * ( cm.z*cm.z + cm.x*cm.x );
	const float zz = integral[4] + integral[5] - mass * ( cm.x*cm.x + cm.y*cm.y );
	const float xy = -( integral[7] - mass * cm.x*cm.y );
	const float yz = -( integral[8] - mass * cm.y*cm.z );
	const float xz = -( integral[9] - mass * cm.z*cm.x );

	mat3 inertia = {
		xx, xy, xz,
		xy, yy, yz,
		xz, yz, zz
	};

	return inertia;
}

/**
* @brief	find the most extreme vertex in a given direction.
*			compute the distance in the direction of an arbitrary vertex in the mesh
			and compare it with the rest of the neightbors until all of them are at 
			a shorter distance
* @param dir	direction to find the vertex
* @return vertex position
*/
vec3 HalfEdgeMesh::hill_climbing( const vec3 dir )
{
	// get arbitrary edge
	auto edge = m_faces[0]->m_edge;
	auto next_edge = edge;

	// O(m^2)
	do
	{
		edge = next_edge;
		float max_distance = dot( m_vertices[edge->vertex], dir );
		auto it_edge = edge;

		do
		{
			// no twin
			if ( it_edge->twin == nullptr )
				break;

			// distance of current adjacent vertex
			float dist = dot( m_vertices[it_edge->twin->vertex], dir );

			// new highest distance
			if ( dist > max_distance )
			{
				next_edge = it_edge->twin;
				max_distance = dist;
			}

			// loop around the edges pointing towards the same vertex
			it_edge = it_edge->twin->prev;

		} while ( it_edge != edge );

		// the edge is the same as in the previous iteration
	} while (edge != next_edge);

	return m_vertices[edge->vertex];
}

/**
* @brief find the most extreme vertex in a given direction (bruteforce)
* @param dir	direction to find the vertex
* @return vertex position
*/
vec3 HalfEdgeMesh::hill_climbing_bruteforce( const vec3 dir )
{
	float max_distance = dot( m_vertices[0u], dir );
	unsigned max_index = 0u;

	// check the distance for every vertex in the mesh O(n)
	for ( unsigned i = 1u; i < m_vertices.size(); i++ )
	{
		float distance = dot( m_vertices[i], dir );
		if ( distance > max_distance )
		{
			max_distance = distance;
			max_index = i;
		}
	}
	return m_vertices[max_index];
}
