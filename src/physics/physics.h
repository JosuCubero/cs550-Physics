/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: physics.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/27/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "half_edge.h"
#include "rigid_body.h"
#include "intersection.h"
#include "solver.h"

#include <vector>

class Physics
{
private:
	Physics() {};

public:
	// instance
	static Physics& get_instance();

	// loop control
	void initialize	();
	void update		( const float dt );
	void exit		();
	void clear		();
	
public:
	// gettors
	const std::vector<RigidBody>&		bodies() const;
	const std::vector<vec4>&			colors() const;
	const std::vector<HalfEdgeMesh*>	meshes() const;

	void set_gravity( const vec3 gravity );

	void add_body( const RigidBody body );

	void show_in_editor();

	void push_object( Ray& ray );

	RigidBody* raycast_scene( Contact& contact, const Ray& ray );
private:
	Contact raycast_body( const Ray& ray, const RigidBody& body ) const;

private:
	std::vector<HalfEdgeMesh*>	m_meshes;
	std::vector<RigidBody>		m_bodies;
	std::vector<vec4>			m_colors;

	Solver* m_collision_solver{ nullptr };

	float m_force_mult;
	vec3 m_gravity;

	bool show_debug_points{ false };
	bool show_debug_colors{ false };
};