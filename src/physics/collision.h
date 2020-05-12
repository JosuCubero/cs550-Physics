/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: collision.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 02/10/2020
----------------------------------------------------------------------------------------------------------*/
#include "rigid_body.h"
#include "half_edge.h"
#include "contact.h"

#include "math_utils.h"


bool overlap_sat( RigidBody& body_A, RigidBody& body_B, ContactManifold& contact_data );

ContactFace has_separating_axis_face( const RigidBody& body_A, const RigidBody& body_B );
ContactEdge has_separating_axis_edge( const RigidBody& body_A, const RigidBody& body_B );

bool create_minkowski_face( const HalfEdge* edge_A, const HalfEdge* edge_B, const RigidBody& body_A, const RigidBody& body_B );
bool is_minkowski_face( const vec3 a, const vec3 b, const vec3 c, const vec3 d );
float edge_distance( const HalfEdge* edge_A, const HalfEdge* edge_B, const RigidBody& body_A, const RigidBody& body_B );

ContactManifold get_contact_manifold( RigidBody& body_A, RigidBody& body_B, ContactFace& contact );
ContactManifold get_contact_manifold( RigidBody& body_A, RigidBody& body_B, const ContactEdge& contact );

float distance_point_plane( const vec3& point, const vec3& plane_normal, const vec3& plane_point );
