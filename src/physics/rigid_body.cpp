/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: rigid_body.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/28/2020
----------------------------------------------------------------------------------------------------------*/
#include "rigid_body.h"


float RigidBody::epsilon = 0.00001f;

/**
* @brief chech vector is 0 with epsilon
* @param v
* @param epsilon
*/
vec3 is_zero( vec3 v, const float epsilon )
{
	vec3 r;

	for ( unsigned i = 0u; i < 3; i++ )
		r[i] = abs( v[i] ) < epsilon ? 0.0f : v[i];

	return r;
}

/**
* @return the model to world matrix
*/
const mat4 RigidBody::model() const
{
	mat4 translation = glm::translate( mat4( 1.0f ), position );
	mat4 rotation = glm::toMat4( rot );
	mat4 scale = glm::scale( scl );

	return translation * rotation * scale;
}

/**
* @brief apply force to a rigid body
* @param force_pos	position of the force (world coordinates)
* @param force_dir	direction of the force (world coordinates)
* @param dt			time the force was applied
*/
void RigidBody::apply_force( const vec3 force_pos, const vec3 force_dir, const float dt )
{
	linear_momentum += force_dir * inv_mass();

	vec3 torque = cross( force_pos - position, force_dir );
	angular_momentum += torque;

	integrate( dt );
}

/**
* @brief add an impulse to the momentum of the body
* @param impulse_pos	point of force
* @param impulse_dir	direction and magnitude of the force
*/
void RigidBody::apply_impulse( const vec3 impulse_pos, const vec3 impulse_dir )
{
	// static body
	if ( mass == 0.0f )
		return;

	// linear component
	linear_momentum += impulse_dir;
	linear_momentum = is_zero( linear_momentum, epsilon );
	linear_velocity = linear_momentum * inv_mass();

	// angular component
	vec3 torque = cross( impulse_pos - position, impulse_dir );
	angular_momentum += torque;

	angular_momentum = is_zero( angular_momentum, epsilon );

	mat3 I_inv = get_oriented_inv_I();
	angular_velocity = I_inv * angular_momentum;
}

/**
* @brief apply angular impulse to the body
* @param impulse_dir
*/
void RigidBody::apply_angular_impulse( const vec3 impulse_dir )
{
	// static body
	if ( mass == 0.0f )
		return;

	// angular component
	angular_momentum += impulse_dir;
	angular_momentum = is_zero( angular_momentum, epsilon );

	mat3 I_inv = get_oriented_inv_I();
	angular_velocity = I_inv * angular_momentum;
}

/**
* @brief update momentum, velocities, position and rotation
* @param dt		delta time
*/
void RigidBody::integrate( const float dt )
{
	//  position
	position += linear_velocity * dt;

	// rotation
	quat delta_rot = 0.5f * quat( 0.0f, angular_velocity.x, angular_velocity.y, angular_velocity.z ) * rot;
	rot = normalize( rot + delta_rot * dt );
}

/**
* @brief get inverse mass of the body
* @return inverse mass
*/
float RigidBody::inv_mass() const
{
	if ( mass != 0.0f )
		return 1.0f / mass;
	// static body
	return 0.0f;
}

/**
* @brief get oriented inverse inertia matrix
* @return mat3
*/
mat3 RigidBody::get_oriented_inv_I() const
{
	return glm::mat3_cast( rot ) * I_inv_body * transpose( glm::mat3_cast( rot ) );;
}
