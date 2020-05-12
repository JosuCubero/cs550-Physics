/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: solver.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 03/11/2020
----------------------------------------------------------------------------------------------------------*/

#pragma once

#include "contact.h"

class Solver
{
public:
	virtual void solve_collision( std::vector<ContactManifold>& contacts, const float dt ) const = 0;
};


class SolverNaive : public Solver
{
public:
	void solve_collision( std::vector<ContactManifold>& contacts, const float dt ) const final;
};

class SolverConstraint : public Solver
{
public:

	void set_iteration_count( const int iterations );
	void set_baumgarte( const float baumgarte );
	void solve_collision( std::vector<ContactManifold>& contacts, const float dt ) const final;

private:
	int m_iterations;
	float m_baumgarte{ 0.0f };

};
