/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: graphics.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/22/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "half_edge.h"
#include "camera.h"
#include "mesh.h"
#include "physics.h"

#include "math_utils.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>


class Graphics
{
private:
	Graphics() {};


public:		// LOOP CONTROL
	static Graphics& get_instance();

	void initialize	();
	void update		();
	void exit		();


public:		// GET - SET
	GLFWwindow*					window() const;
	const Camera&				camera() const;
	const std::vector<Mesh>&	meshes() const;

	void set_camera( const Camera camera );

public:		// PUBLIC METHODS
	vec3 mouse_pos() const;

	void debug_render( const HalfEdgeMesh* mesh,
					   const vec3 pos, const vec3 scl, const quat rot,
					   const vec4 color );

private:	// PRIVATE METHODS
	void		initialize_glfw	() const;
	GLFWwindow* create_window	();
	void		initialize_glad	() const;
	unsigned	compile_shader	( const char* file_name, GLenum shader_type ) const;
	unsigned	create_program	( const char* vertex_shader, const char* fragment_shader ) const;
	void		load_meshes		();

private:	// MEMBERS
	GLFWwindow* m_window;
	unsigned	m_program;
	unsigned	m_vao;

	Camera		m_camera;
	mat4		m_perspective;

	int m_width{ 0 };
	int m_height{ 0 };

	unsigned m_vertex_buffer;
	unsigned m_index_buffer;

	std::vector<Mesh> m_meshes;
};
