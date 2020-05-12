/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: physics.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/27/2020
----------------------------------------------------------------------------------------------------------*/
#include "physics.h"

#include "collision.h"
#include "camera.h"
#include "graphics.h"

#include "math_utils.h"
#include <imgui/imgui.h>
#include <string>


/**
* @brief get instance of the singleton
* @return instance
*/
Physics & Physics::get_instance()
{
	static Physics instance{};
	return instance;
}

/**
* @brief initialize physics
*/
void Physics::initialize()
{
	auto& meshes = Graphics::get_instance().meshes();

	// create the half edge meshes from the physical meshes
	for ( unsigned i = 0; i < meshes.size(); i++ )
	{
		HalfEdgeMesh* phy_mesh = new HalfEdgeMesh;
		phy_mesh->set_render_mesh_id( i );
		phy_mesh->add_vertices( meshes[i].vertices );
		for ( unsigned j = 0; j < meshes[i].indices.size(); j++ )
			phy_mesh->add_face(	meshes[i].indices[j].x,
								meshes[i].indices[j].y,
								meshes[i].indices[j].z );

		phy_mesh->link_twins();
		phy_mesh->merge_faces();
		phy_mesh->set_indices();

		m_meshes.push_back( phy_mesh );
	}

	//SolverNaive* solver = new SolverNaive;
	SolverConstraint* solver = new SolverConstraint;
	solver->set_iteration_count( 20 );
	solver->set_baumgarte( 0.2f );
	m_collision_solver = reinterpret_cast<Solver*>( solver );


	m_force_mult = 0.5f;
	m_gravity = { 0.0f, -10.0f, 0.0f };
}

/**
* @brief update physics
*/
void Physics::update( const float dt )
{
	// clean colors DEBUG
	if ( show_debug_colors == true )
		for ( auto& color : m_colors )
			color = vec4( 1.0f, 0.0f, 0.0f, 1.0f );
	else
		for ( auto& color : m_colors )
			color = vec4( 0.0f, 0.0f, 0.0f, 1.0f );



	// check collisions
	std::vector<ContactManifold> contacts;

	if ( m_bodies.size() > 1u )
	{
		// check every body with the rest of bodies
		for ( unsigned i = 0u; i < m_bodies.size() - 1u; i++ )
		{
			for ( unsigned j = i + 1u; j < m_bodies.size(); j++ )
			{
				ContactManifold contact;

				if ( overlap_sat( m_bodies[i], m_bodies[j], contact ) )
				{
					contacts.push_back( contact );

					// change colors DEBUG
					if ( show_debug_colors == true )
					{
						m_colors[i] = vec4( 0.0f, 1.0f, 0.0f, 1.0f );
						m_colors[j] = vec4( 0.0f, 1.0f, 0.0f, 1.0f );
					}
				}
			}
		} // O(n^2)
	}


	// apply gravity
	for ( auto& body : m_bodies )
		body.apply_impulse( body.position, m_gravity * dt * body.mass );


	// DEBUG	
	if ( show_debug_points == true )
	{
		for ( auto contact : contacts )
		{
			// render contact points DEBUG
			for ( auto point : contact.points )
			{
				Graphics::get_instance().debug_render( m_meshes[0u], point.point_B, vec3( 0.05f ), quat(), vec4( 0.0f, 0.5f, 0.8f, 1.0f ) );
				Graphics::get_instance().debug_render( m_meshes[0u], point.point_A, vec3( 0.05f ), quat(), vec4( 0.0f, 0.5f, 0.8f, 1.0f ) );
			}
		}
	}


	// apply contact solver
	m_collision_solver->solve_collision( contacts, dt );


	// update velocities and position of bodies
	for ( auto& body : m_bodies )
		body.integrate( dt );
}

/**
* @brief exit physics
*/
void Physics::exit()
{
	for ( auto mesh : m_meshes )
		delete mesh;

	m_meshes.clear();
	clear();

	delete m_collision_solver;
}

/**
* @brief clear bodies
*/
void Physics::clear()
{
	m_bodies.clear();
}






/**
* @brief get bodies
* @return bodies
*/
const std::vector<RigidBody>& Physics::bodies() const
{
	return m_bodies;
}

/**
* @brief get colors of bodies
* @return colors
*/
const std::vector<vec4>& Physics::colors() const
{
	return m_colors;
}

/**
* @brief get physical meshes
* @return meshes
*/
const std::vector<HalfEdgeMesh*> Physics::meshes() const
{
	return m_meshes;
}

/**
* @brief change gravity value
* @param gravity	new gravity
*/
void Physics::set_gravity( const vec3 gravity )
{
	m_gravity = gravity;
}

/**
* @brief add a new rigid body
* @param body
*/
void Physics::add_body( const RigidBody body )
{
	m_bodies.push_back( body );
	if ( show_debug_colors == true)
		m_colors.push_back( vec4( 1.0f, 0.0f, 0.0f, 1.0f ) );
	else
		m_colors.push_back( vec4( 0.0f, 0.0f, 0.0f, 1.0f ) );
}






/**
* @ brief push an object in the scene given a ray
*/
void Physics::push_object( Ray & ray )
{
	Contact contact;
	RigidBody* body = raycast_scene( contact, ray );

	// update forces in case of collision
	if ( contact.time != -1.0f )
	{
		vec3 force = normalize( contact.position - Graphics::get_instance().camera().position() );
		body->apply_impulse( contact.position, force * m_force_mult );
	}
}

/**
* @brief throw a raycast through the scene
* @param collided_body	return pointer to the collided body
* @param ray			ray to throw
* @return contant information
*/
RigidBody* Physics::raycast_scene( Contact& contact, const Ray& ray )
{
	contact.time = -1.0f;
	RigidBody* result = nullptr;

	// raycast bodies
	for ( auto& body : m_bodies )
	{
		Contact temp = raycast_body( ray, body );
		if ( contact.time == -1.0f || temp.time != -1.0f && temp.time < contact.time )
		{
			contact = temp;
			result = &body;
		}
	}

	return result;
}

/**
* @brief raycast against a body
*/
Contact Physics::raycast_body( const Ray& ray, const RigidBody& body ) const
{
	// transform the ray to body space
	mat4 model		= body.model();
	mat4 model_inv	= inverse( model );

	Ray body_ray = ray;
	body_ray.origin		= vec3( model_inv * vec4{ ray.origin, 1.0f } );
	body_ray.direction	= vec3( model_inv * vec4{ ray.direction, 0.0f } );

	// check ray against body
	Contact result	= intersection_ray_polyhedra( body_ray, body.mesh );

	// translate the contact information back to world space
	result.position = vec3( model * vec4( result.position, 1.0f ) );
	result.normal	= vec3( model * vec4( result.normal, 0.0f ) );

	return result;
}

/**
* @brief show the bodies in the editor
*/
void Physics::show_in_editor()
{
	ImGui::Begin( "Physics" );
	{
		// physics
		ImGui::DragFloat3( "Gravity", &m_gravity.x, 0.01f );
		ImGui::Checkbox( "Debug Points", &show_debug_points );
		ImGui::Checkbox( "Debug Colors", &show_debug_colors );

		// bodies
		for ( unsigned i = 0; i < m_bodies.size(); i++ )
		{
			std::string name = "RigidBody " + std::to_string( i );
			if ( ImGui::TreeNode( name.c_str() ) )
			{
				vec3 euler = glm::degrees( glm::eulerAngles( m_bodies[i].rot ) );

				ImGui::DragFloat3( "Position", &m_bodies[i].position.x, 0.01f );
				ImGui::DragFloat3( "Scale", &m_bodies[i].scl.x, 0.01f );
				ImGui::DragFloat3( "Rotation", &euler.x, 0.01f );
				ImGui::DragFloat3( "Velocity", &m_bodies[i].linear_velocity.x, 0.01f );
				ImGui::DragFloat3( "Angular Velocity", &m_bodies[i].angular_velocity.x, 0.01f );
				ImGui::DragFloat3( "Linear Momentum", &m_bodies[i].linear_momentum.x, 0.01f );
				ImGui::DragFloat3( "Angular Momentum", &m_bodies[i].angular_momentum.x, 0.01f );
				ImGui::DragFloat( "Mass", &m_bodies[i].mass, 0.01f );

				ImGui::TreePop();

				

				m_bodies[i].rot = normalize( quat( glm::radians( euler ) ) );
			}

		}
	} ImGui::End();
}
