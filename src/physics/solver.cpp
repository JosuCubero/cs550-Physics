/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: solver.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 03/11/2020
----------------------------------------------------------------------------------------------------------*/

#include "solver.h"
#include "rigid_body.h"


vec3 point_velocity( const RigidBody* body, const vec3 point );

/**
* DEPRECATED
* @brief solve the collision applying forces to the rigid bodies
* @param contacts
*/
void SolverNaive::solve_collision( std::vector<ContactManifold>& contacts, const float dt ) const
{
	/*
		TO DO: implement naive collision solver using Baraff's notes
		- https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf
	*/

	const float restitution = 1.0f;


	/*for ( auto& contact : contacts )
	{
		for ( unsigned i = 0u; i < contact.points.size(); i++ )
		{
			auto& contact_point = contact.points[i];
			vec3 point = contact_point.point;

			// linear velocity of the contact points
			vec3 padot = point_velocity( contact.body_A, point );
			vec3 pbdot = point_velocity( contact.body_B, point );
			vec3 normal = contact.normal;

			// position of the contact point respect to the body
			vec3 ra = point - contact.body_A->position;
			vec3 rb = point - contact.body_B->position;

			// compute relative velocity
			float vrel = glm::dot( normal, padot - pbdot );
			float numerator = -( 1 + restitution ) * vrel;

			float e_mass_1 = contact.body_A->inv_mass();
			float e_mass_2 = contact.body_B->inv_mass();
			float e_mass_3 = glm::dot( normal, glm::cross( contact.body_A->I_inv_body * glm::cross( ra, normal ), ra ) );
			float e_mass_4 = glm::dot( normal, glm::cross( contact.body_B->I_inv_body * glm::cross( rb, normal ), rb ) );

			// compute impulse
			float j = numerator / ( e_mass_1 + e_mass_2 + e_mass_3 + e_mass_4 );

			contact_point.impulse.x = j;

			j /= contact.points.size();
			vec3 force_dir = j * normal;

			// apply forces
			contact.body_A->apply_impulse( point,  force_dir );
			contact.body_B->apply_impulse( point, -force_dir );
		}

		// update velocities
		contact.body_A->linear_velocity = contact.body_A->linear_momentum * contact.body_A->inv_mass();
		contact.body_B->linear_velocity = contact.body_B->linear_momentum * contact.body_B->inv_mass();

		contact.body_A->angular_velocity = contact.body_A->I_inv_body * contact.body_A->angular_momentum;
		contact.body_B->angular_velocity = contact.body_B->I_inv_body * contact.body_B->angular_momentum;
	}*/
}




/**
* @brief set the iterations for the solver
* @param iterations
*/
void SolverConstraint::set_iteration_count( const int iterations )
{
	m_iterations = iterations;
}

/**
* @brief set the baumgarte bias for the solver
* @param baumgarte
*/
void SolverConstraint::set_baumgarte( const float baumgarte )
{
	m_baumgarte = baumgarte;
}

/**
* @brief solve the collision applying forces to the rigid bodies
* @param contacts
*/
void SolverConstraint::solve_collision( std::vector<ContactManifold>& contacts, const float dt ) const
{
	const float depth_threshold = 0.01f;
	const float velocity_threshold = 1.0f;

	for ( int i = 0u; i < m_iterations; i++ )
	{
		for ( int j = 0; j < contacts.size(); j++ )
		{
			auto& contact = contacts[j];

			const vec3 n = contact.normal;
			const vec3 cross_vec = { n.y, n.z, -n.x };
			const vec3 u = cross( n, cross_vec );
			const vec3 v = cross( n, u );

			// constant terms of effective mass
			const float e_mass_1 = contact.body_A->inv_mass();
			const float e_mass_2 = contact.body_B->inv_mass();

			float total_impulse = 0.0f;

			const float restitution = contact.body_A->restitution * contact.body_B->restitution;

			for ( unsigned k = 0u; k < contact.points.size(); k++ )
			{
				auto& contact_point = contact.points[k];

				const vec3 point_A = contact_point.point_A;
				const vec3 point_B = contact_point.point_B;

				// linear velocity of the contact points
				const vec3 va = point_velocity( contact.body_A, point_A );
				const vec3 vb = point_velocity( contact.body_B, point_B );

				// position of the contact point respect to the body
				const vec3 ra = point_A - contact.body_A->position;
				const vec3 rb = point_B - contact.body_B->position;

				// terms of inverse mass
				const vec3 ra_n = cross( ra, n );
				const vec3 rb_n = cross( rb, n );

				// effective mass
				const float e_mass_3 = glm::dot( ra_n, contact.body_A->I_inv_body * ra_n );
				const float e_mass_4 = glm::dot( rb_n, contact.body_B->I_inv_body * rb_n );

				const float effective_mass = 1.0f / contact.points.size() * 1.0f / ( e_mass_1 + e_mass_2 + e_mass_3 + e_mass_4 );

				const float Jvn = dot( vb - va, n ); // collision

				if ( i == 0 )
					contact_point.Jv0 = Jvn;

				const float depth_bias = -m_baumgarte * ( contact_point.depth - depth_threshold ) / dt; 
				
				const float restitution_bias = contact_point.Jv0 < -velocity_threshold ? restitution * contact_point.Jv0 : 0.0f;

				const float old_impulse = contact_point.impulse;

				// acumulate impulse
				contact_point.impulse += effective_mass * -( Jvn + depth_bias + restitution_bias );;
				contact_point.impulse = glm::max( 0.0f, contact_point.impulse );
				total_impulse += contact_point.impulse;


				// apply impulse
				const vec3 impulse_dir = n * ( contact_point.impulse - old_impulse );

				contact.body_A->apply_impulse( point_A, -impulse_dir );
				contact.body_B->apply_impulse( point_B,  impulse_dir );
			}

			// point average
			vec3 avg_point_A{ 0.0f };
			vec3 avg_point_B{ 0.0f };
			for ( auto& point : contact.points )
			{
				avg_point_A += point.point_A;
				avg_point_B += point.point_B;
			}
			avg_point_A /= contact.points.size();
			avg_point_B /= contact.points.size();

			// position of the contact point respect to the body
			const vec3 ra = avg_point_A - contact.body_A->position;
			const vec3 rb = avg_point_B - contact.body_B->position;

			// terms of inverse mass
			const vec3 ra_u = cross( ra, u );
			const vec3 rb_u = cross( rb, u );
			const vec3 ra_v = cross( ra, v );
			const vec3 rb_v = cross( rb, v );

			// effective mass
			const float e_mass_5 = glm::dot( ra_u, contact.body_A->I_inv_body * ra_u );
			const float e_mass_6 = glm::dot( rb_u, contact.body_B->I_inv_body * rb_u );

			const float e_mass_7 = glm::dot( ra_v, contact.body_A->I_inv_body * ra_v );
			const float e_mass_8 = glm::dot( rb_v, contact.body_B->I_inv_body * rb_v );

			const float effective_mass_u = 1.0f / ( e_mass_1 + e_mass_2 + e_mass_5 + e_mass_6 );
			const float effective_mass_v = 1.0f / ( e_mass_1 + e_mass_2 + e_mass_7 + e_mass_8 );
			const float effective_mass_t = 1.0f / ( dot( n, contact.body_A->get_oriented_inv_I() * n ) + dot( n, contact.body_B->get_oriented_inv_I() * n ) );

			// body angular velocity
			const vec3 wa = contact.body_A->angular_velocity;
			const vec3 wb = contact.body_B->angular_velocity;


			// linear velocity of the average contact points
			const vec3 va = point_velocity( contact.body_A, avg_point_A );
			const vec3 vb = point_velocity( contact.body_B, avg_point_B );

			// jacovian for each axis
			const float Jvu = dot( vb - va, u );
			const float Jvv = dot( vb - va, v );
			const float Jvt = dot( wb - wa, n );

			const float impulse_u_old = contact.impulse_u;
			const float impulse_v_old = contact.impulse_v;
			const float impulse_t_old = contact.impulse_t;


			contact.impulse_u += effective_mass_u * -Jvu;
			contact.impulse_v += effective_mass_v * -Jvv;
			contact.impulse_t += effective_mass_t * -Jvt;	// friction


			const float friction = contact.body_A->friction * contact.body_B->friction * total_impulse;
			contact.impulse_u = glm::clamp( contact.impulse_u, -friction, friction );
			contact.impulse_v = glm::clamp( contact.impulse_v, -friction, friction );
			contact.impulse_t = glm::clamp( contact.impulse_t, -friction, friction );

			const vec3 impulse_dir = u * ( contact.impulse_u - impulse_u_old ) +
									 v * ( contact.impulse_v - impulse_v_old );

			contact.body_A->apply_impulse( avg_point_A, -impulse_dir );
			contact.body_B->apply_impulse( avg_point_B, impulse_dir );

			// twist
			const vec3 impulse_twist = n * ( contact.impulse_t - impulse_t_old );

			contact.body_A->apply_angular_impulse( -impulse_twist );
			contact.body_B->apply_angular_impulse( impulse_twist );
		}
	}
}





/**
* @brief compute the velocity of a particle in a body
*/
vec3 point_velocity( const RigidBody* body, const vec3 point )
{
	vec3 r = point - body->position;
	return body->linear_velocity + glm::cross( body->angular_velocity, r );
}

