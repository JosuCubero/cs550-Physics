/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: camera.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/25/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "math_utils.h"

using namespace glm;

class Camera
{
public:
	Camera() {}

	void initialize();
	void update();

	mat4 world_to_cam() const;
	vec3 position() const;

	void set_position	( const vec3 pos );
	void set_view	( const vec3 dir );
	void set_up			( const vec3 up  );

private:
	vec2 cursor_pos();

private:
	vec3 m_position{ 0.0f, 0.0f, 5.0f };

	vec3 m_view{ 0.0f, 0.0f, -1.0f };
	vec3 m_up{ 0.0f, 1.0f,  0.0f };
	vec3 m_right{ 1.0f, 0.0f,  0.0f };

	mat4 m_world_to_cam{ 1 };

	vec2 m_mouse_pos{ 0 };
	vec2 m_mouse_prev_pos{ 0 };
};
