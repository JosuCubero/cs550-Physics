/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: rigid_body.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/28/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "half_edge.h"
#include "math_utils.h"


struct RigidBody
{
	static float epsilon; 

	HalfEdgeMesh* mesh;

	float mass;

	mat3 I_body;
	mat3 I_inv_body;

	vec3 position{ 0.0f, 0.0f, 0.0f };
	quat rot = quat();
	vec3 scl{ 1.0f, 1.0f, 1.0f };
	vec3 linear_momentum{ 0.0f, 0.0f, 0.0f };
	vec3 angular_momentum{ 0.0f, 0.0f, 0.0f };
	
	vec3 linear_velocity{ 0.0f, 0.0f, 0.0f, };
	vec3 angular_velocity{ 0.0f, 0.0f, 0.0f, };

	float restitution{ 0.0f };
	float friction{ 0.0f };


	const mat4 model() const;
	void apply_force( const vec3 force_pos, const vec3 force_dir, const float dt );
	void apply_impulse( const vec3 impulse_pos, const vec3 impulse_dir );
	void apply_angular_impulse( const vec3 impulse_dir );
	void integrate( const float dt );
	float inv_mass() const;
	mat3 get_oriented_inv_I() const;
};