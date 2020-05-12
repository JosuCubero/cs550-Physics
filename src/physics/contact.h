/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: contact.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 03/11/2020
----------------------------------------------------------------------------------------------------------*/

#pragma once

#include "math_utils.h"
#include <vector>

struct HalfEdge;
struct RigidBody;

struct ContactFace
{
	unsigned face_id;
	float separation{ -std::numeric_limits<float>::max() };
};

struct ContactEdge
{
	float separation{ -std::numeric_limits<float>::max() };
	HalfEdge* edge_A{ nullptr };
	HalfEdge* edge_B{ nullptr };
};

struct ContactPoint
{
	vec3	point_B;
	vec3	point_A;
	float	depth{ 0.0f };
	float 	impulse{ 0.0f };
	float	Jv0{ 0.0f };
};

struct ContactManifold
{
	std::vector<ContactPoint> points;
	vec3 normal;				// normal of separation in world space

	RigidBody* body_A;
	RigidBody* body_B;

	float impulse_u{ 0.0f };
	float impulse_v{ 0.0f };
	float impulse_t{ 0.0f };
};