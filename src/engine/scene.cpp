/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: scene.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 02/22/2020
----------------------------------------------------------------------------------------------------------*/
#include "scene.h"

#include "physics.h"
#include "graphics.h"

#include <fstream>
#include <iostream>

/**
* @brief clear
*/
void Scene::clear()
{
	Physics::get_instance().clear();
}

/**
* @brief scene constructor
* @param filename	scene file
*/
Scene::Scene( const char* filename )
{
	load_scene( filename );
}

/**
* @brief load the scene from a file
* @param filename
*/
void Scene::load_scene( const char* filename )
{
	// clear data if any
	clear();

	std::ifstream file;
	file.open( filename );

	if ( !file )
	{
		std::cout << "couldn't open file " << filename << std::endl;
		return;
	}

	// copy the file stream to the stream buffer
	std::string file_data;
	file.seekg( 0, std::ios::end );
	file_data.reserve( file.tellg() );
	file.seekg( 0, std::ios::beg );
	file_data.assign( std::istreambuf_iterator<char>( file ), std::istreambuf_iterator<char>() );

	// close the file
	file.close();

	// proccess data
	while ( file_data.empty() == false )
	{
		// proccess the line
		read_line( file_data );
	}
}

/**
* @brief proccess a line of the scene to load
* @param line	line to process
*/
void Scene::read_line( std::string& line )
{
	// skip comments
	if ( line.rfind( '#', 0u ) == 0u )
	{
		line = line.substr( line.find( '\n' ) + 1u );
		return;
	}

	if ( line.rfind( "PHYSICS", 0u ) == 0u )
	{
		auto& physics = Physics::get_instance();
		physics.set_gravity( read_vector( line ) );
	}

	// read cube
	if ( line.rfind( "CUBE", 0u ) == 0u )
	{
		add_cube( line );
		return;
	}

	// read cylinder
	else if ( line.rfind( "CYLINDER", 0u ) == 0u )
	{
		add_cylinder( line );
		return;
	}

	// read icosahedron
	else if ( line.rfind( "ICOSAHEDRON", 0u ) == 0u )
	{
		add_icosahedron( line );
		return;
	}

	// read octohedron
	else if ( line.rfind( "OCTOHEDRON", 0u ) == 0u )
	{
		add_octohedron( line );
		return;
	}

	// read sphere
	else if ( line.rfind( "SPHERE", 0u ) == 0u )
	{
		add_sphere( line );
		return;
	}

	// read camera
	else if ( line.rfind( "CAMERA", 0u ) == 0u )
	{
		Camera camera;
		camera.set_position( read_vector( line ) );
		camera.set_view( read_vector( line ) );
		camera.set_up( read_vector( line ) );
		camera.initialize();
		Graphics::get_instance().set_camera( camera );
		return;
	}
	// empty lines
	else
	{
		line = line.substr( 1u );
	}
}

/**
* @brief read the data of a rigid body from an string
* @param data
*/
RigidBody Scene::read_body( std::string& data )
{
	RigidBody body;
	body.position = read_vector( data );
	body.scl = read_vector( data );
	body.rot = quat( glm::radians( read_vector( data ) ) );
	body.mass = read_float( data );
	body.restitution = read_float( data );
	body.friction = read_float( data );
	return body;
}

/**
* @brief read a vector from a string
* @param data	string to read from
* @return vector
*/
vec3 Scene::read_vector( std::string & data )
{
	vec3 vec;

	vec.x = read_float( data, '(', ',' );	// x value
	vec.y = read_float( data, ',', ',' );	// y value
	vec.z = read_float( data, ',', ')' );	// z value

	return vec;
}

/**
* @brief read a float value from a string
* @param data		string to read from
* @param prev_char	character previous to the value
* @param post_char	character right after the value
* @return float
*/
float Scene::read_float( std::string& data, const char prev_char, const char post_char )
{
	float result;

	size_t start = data.find( prev_char );
	data = data.substr( start + 1u );
	size_t end = data.find( post_char );

	result = static_cast< float >( std::atof( data.substr( 0u, end ).c_str() ) );
	data = data.substr( end );

	return result;
}

/**
* @brief read a float value from a string
* @param data		string to read from
* @return float
*/
float Scene::read_float( std::string& data )
{
	float result;

	size_t new_line = data.find( '\n' );
	size_t space = data.find( ' ' );

	size_t start = new_line < space ? new_line : space;
	data = data.substr( start + 1u );

	new_line = data.find( '\n' );
	space = data.find( ' ' );

	size_t end = new_line < space ? new_line : space;

	result = static_cast<float>( std::atof( data.substr( 0u, end ).c_str() ) );
	data = data.substr( end );

	return result;
}





/**
* @brief add a cube rigid body
*/
void Scene::add_cube( std::string& data )
{
	RigidBody body = read_body( data );
	body.mesh = Physics::get_instance().meshes()[0u];

	//body.I_body = mat3( 0.0f );
	body.I_body = body.mesh->compute_intertia_tensor();
						/*mat3( 0.5f, 0.0f,  0.0f,
						0.0f, 0.25f, 0.0f,
						0.0f, 0.0f,  0.5f );*/
	
	/* change intertia tensor I_new = Iold * ( mass_new / mass_old ) */
	body.I_body *= 1.0f * body.inv_mass();

	if ( body.I_body == mat3( 0.0f ) )
		body.I_inv_body = mat3( 0.0f );
	else
		body.I_inv_body = inverse( body.I_body );

	Physics::get_instance().add_body( body );
}

/**
* @brief add a cylinder rigid body
*/
void Scene::add_cylinder( std::string& data )
{
	RigidBody body = read_body( data );
	body.mesh = Physics::get_instance().meshes()[1u];

	body.I_body = mat3( 1.0f / 6.0f, 0.0f,		  0.0f,
						0.0f,		 1.0f / 6.0f, 0.0f,
						0.0f,		 0.0f,		  1.0f / 6.0f );

	body.I_inv_body = inverse( body.I_body );

	Physics::get_instance().add_body( body );
}

/**
* @brief add a icosahedron rigid body
*/
void Scene::add_icosahedron( std::string& data )
{
	RigidBody body = read_body( data );
	body.mesh = Physics::get_instance().meshes()[2u];

	const float I = glm::pi<float>() / 15.0f;
	body.I_body = mat3( I,	  0.0f, 0.0f,
						0.0f, I,	0.0f,
						0.0f, 0.0f, I );

	body.I_inv_body = inverse( body.I_body );

	Physics::get_instance().add_body( body );
}

/**
* @brief add a octohedron rigid body
*/
void Scene::add_octohedron( std::string& data )
{
	RigidBody body = read_body( data );
	body.mesh = Physics::get_instance().meshes()[3u];

	const float I = glm::pi<float>() / 15.0f;
	body.I_body = mat3( I,	  0.0f, 0.0f,
						0.0f, I,	0.0f,
						0.0f, 0.0f, I );

	body.I_inv_body = inverse( body.I_body );

	Physics::get_instance().add_body( body );
}

/**
* @brief add a sphere rigid body
*/
void Scene::add_sphere( std::string& data )
{
	RigidBody body = read_body( data );
	body.mesh = Physics::get_instance().meshes()[4u];

	const float I = glm::pi<float>() / 15.0f;
	body.I_body = mat3( I,	  0.0f, 0.0f,
						0.0f, I,	0.0f,
						0.0f, 0.0f, I );

	body.I_inv_body = inverse( body.I_body );

	Physics::get_instance().add_body( body );
}
