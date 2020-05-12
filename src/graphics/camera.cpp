/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: camera.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/25/2020
----------------------------------------------------------------------------------------------------------*/
#include "camera.h"

#include "graphics.h"

#include <GLFW/glfw3.h>


/**
* @brief initialize camera parameters
*/
void Camera::initialize()
{
	m_right = cross( m_view, m_up );
	m_world_to_cam = glm::lookAt( m_position, m_position + m_view, m_up );
	m_mouse_pos = cursor_pos();
	m_mouse_prev_pos = m_mouse_pos;
}

/**
* @brief update the camera
*/
void Camera::update()
{
	auto window = Graphics::get_instance().window();
	const float linear_speed = 0.1f;
	const float turn_speed = 0.2f;

	m_mouse_pos = cursor_pos();

	// move camera
	if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_2 ) )
	{
		// position
		if ( glfwGetKey( window, GLFW_KEY_W ) )
			m_position += m_view * linear_speed;
		if ( glfwGetKey( window, GLFW_KEY_S ) )
			m_position -= m_view * linear_speed;
		if ( glfwGetKey( window, GLFW_KEY_D ) )
			m_position += m_right * linear_speed;
		if ( glfwGetKey( window, GLFW_KEY_A ) )
			m_position -= m_right * linear_speed;
		if ( glfwGetKey( window, GLFW_KEY_E ) )
			m_position.y += linear_speed;
		if ( glfwGetKey( window, GLFW_KEY_Q ) )
			m_position.y -= linear_speed;

		// rotation
		vec2 delta = m_mouse_pos - m_mouse_prev_pos;
		vec4 view	= vec4( m_view, 0.0f );
		vec4 up		= vec4( m_up, 0.0f );
		const vec3 y_axis{ 0.0f, 1.0f, 0.0f };
		
		view	= rotate( mat4( 1.0f ), radians( turn_speed ) * -delta.y, m_right ) * view;
		up		= rotate( mat4( 1.0f ), radians( turn_speed ) * -delta.y, m_right ) * up;
		
		// avoid camera upside down
		if ( dot( vec3( up ), y_axis ) < 0.1f )
		{
			view	= vec4( m_view, 0.0f );
			up		= vec4( m_up, 0.0f );
		}
		
		// update camera vectors
		view	= rotate( mat4( 1.0f ), radians( turn_speed ) * -delta.x, y_axis )  * view;
		up		= rotate( mat4( 1.0f ), radians( turn_speed ) * -delta.x, y_axis )  * up;

		m_view	= vec3( view );
		m_up	= vec3( up );
		m_right = cross( m_view, m_up );

		// update world to camera matrix
		m_world_to_cam = glm::lookAt( m_position, m_position + m_view, m_up );
	}

	m_mouse_prev_pos = m_mouse_pos;
}





/**
* @brief get world to camera matrix
* @return world to camera
*/
mat4 Camera::world_to_cam() const
{
	return m_world_to_cam;
}

/**
* @brief get position
* @return position
*/
vec3 Camera::position() const
{
	return m_position;
}

/**
* @brief set position
* @param pos
*/
void Camera::set_position( const vec3 pos )
{
	m_position = pos;

}

/**
* @brief set view vector
* @param dir
*/
void Camera::set_view( const vec3 dir )
{
	m_view = normalize( dir );
}

/**
* @brief set up vector
* @param up
*/
void Camera::set_up( const vec3 up )
{
	m_up = normalize( up );
}

/**
* @brief get the position of the cursor in screen
*/
vec2 Camera::cursor_pos()
{
	double x, y;
	glfwGetCursorPos( Graphics::get_instance().window(), &x, &y );
	return vec2{ static_cast< float >( x ), static_cast< float >( y ) };
}
