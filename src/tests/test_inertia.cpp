/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: test_inertia.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 03/08/2020
----------------------------------------------------------------------------------------------------------*/
#include <gtest/gtest.h>

#include "half_edge.h"
#include "mesh.h"

#include "math_utils.h"


TEST( inertia, cube_inertia_tensor )
{
	// load OBJs
	Mesh cube_mesh = load_obj("../resources/meshes/cube.obj");

	HalfEdgeMesh cube;
	cube.add_vertices(cube_mesh.vertices);
	for (unsigned i = 0; i < cube_mesh.indices.size(); i++)
		cube.add_face(cube_mesh.indices[i].x,
			cube_mesh.indices[i].y,
			cube_mesh.indices[i].z);

	cube.link_twins();
	cube.merge_faces();
	cube.set_indices();
	mat3 inertia_tensor = cube.compute_intertia_tensor();
	mat3 analytic( 1.0f / 6.0f, 0.0f, 0.0f,
				   0.0f, 1.0f / 6.0f, 0.0f,
				   0.0f, 0.0f, 1.0f / 6.0f );

	ASSERT_EQ( inertia_tensor, analytic );
}

/*
TEST( inertia, cylinder_inertia_tensor )
{
	// load OBJs
	Mesh cylinder_mesh = load_obj( "../resources/meshes/cylinder.obj" );

	HalfEdgeMesh cylinder;
	cylinder.add_vertices( cylinder_mesh.vertices );
	for ( unsigned i = 0; i < cylinder_mesh.indices.size(); i++ )
		cylinder.add_face( cylinder_mesh.indices[i].x,
			cylinder_mesh.indices[i].y,
			cylinder_mesh.indices[i].z );

	cylinder.link_twins();
	cylinder.merge_faces();
	cylinder.set_indices();
	mat3 inertia_tensor = cylinder.compute_intertia_tensor();

	const float val = 1.0f / 12.0f * ( 3.0f + 16.0f );

	mat3 analytic( val,	 0.0f, 0.0f,
				   0.0f, val,  0.0f,
				   0.0f, 0.0f, val  );

	ASSERT_EQ( inertia_tensor, analytic );
}
*/