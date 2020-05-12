/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: graphics.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/22/2020
----------------------------------------------------------------------------------------------------------*/
#include "graphics.h"

#include "editor.h"
#include "math_utils.h"

#include <string>
#include <fstream>
#include <iostream>

/**
* @brief get instance of the singleton
* @return instance
*/
Graphics& Graphics::get_instance()
{
	static Graphics instance{};
	return instance;
}

/**
* @brief initialize graphcis
*/
void Graphics::initialize()
{
	// initialize glfw
	initialize_glfw();
	m_window = create_window();
	initialize_glad();

	// create shader program
	m_program = create_program( "../resources/shaders/color.vert", "../resources/shaders/color.frag" );

	// load meshes
	load_meshes();

	// generate buffers
	glGenVertexArrays( 1, &m_vao );

	glGenBuffers( 1, &m_vertex_buffer );
	glGenBuffers( 1, &m_index_buffer );

	// enable blend
	glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

	// perspective matrix transformation
	m_perspective = glm::perspective( 45.0f, 16.0f / 9.0f, 0.1f, 100.0f );
}

/**
* @brief update graphics
*/
void Graphics::update()
{
	// set window context
	int display_w, display_h;
	glfwMakeContextCurrent( m_window );
	glfwGetFramebufferSize( m_window, &display_w, &display_h );

	// update camera
	m_camera.update();

	// render
	auto& bodies = Physics::get_instance().bodies();
	auto& colors = Physics::get_instance().colors();
	for ( unsigned i = 0u; i < bodies.size(); i++ )
		debug_render( bodies[i].mesh, bodies[i].position, bodies[i].scl, bodies[i].rot, colors[i] );

	Editor::get_instance().render();

	glfwMakeContextCurrent( m_window );
	glfwSwapBuffers( m_window );

	// clear buffer for next frame
	glClearColor( 0.2f, 0.2f, 0.2f, 1 );
	glClear( GL_COLOR_BUFFER_BIT );
}


/**
* @brief exit graphics
*/
void Graphics::exit()
{
	glDeleteBuffers( 1, &m_vertex_buffer );
	glDeleteBuffers( 1, &m_index_buffer );
	glDeleteVertexArrays( 1, &m_vao );
	glDeleteProgram( m_program );
	glfwDestroyWindow( m_window );
	glfwTerminate();
}





/**
* @brief get window
* @return window
*/
GLFWwindow * Graphics::window() const
{
	return m_window;
}

/**
* @brief get the camera
* @return camera
*/
const Camera & Graphics::camera() const
{
	return m_camera;
}

/**
* @brief get the meshes
*/
const std::vector<Mesh>& Graphics::meshes() const
{
	return m_meshes;
}

/**
* @brief set the camera
* @param camera
*/
void Graphics::set_camera( const Camera camera )
{
	m_camera = camera;
}

/**
* @brief get the position of the mouse in world coordinates
* @return mouse pos
*/
vec3 Graphics::mouse_pos() const
{
	double x, y;

	// cursor position in screen
	glfwGetCursorPos( m_window, &x, &y );

	// transform to world coordinates
	vec2 screen( static_cast<float>( x ), static_cast<float>( y ) );
	vec2 ndc = { 2.0f * ( screen.x / static_cast<float>( m_width ) ) - 1.0f, -( 2.0f * ( screen.y / static_cast<float>( m_height ) ) - 1.0f ) };

	mat4 perspective_to_world = inverse( m_perspective * m_camera.world_to_cam() );
	vec4 world = perspective_to_world * vec4( ndc, 1.0f, 1.0f );
	world /= world.w;

	return vec3( world );
}




/**
* @brief render a mesh of lines
* @param mesh
* @param translation
* @param scale
* @param rotation
* @param color
*/
void Graphics::debug_render( const HalfEdgeMesh* mesh, const vec3 pos, const vec3 scl, const quat rot, const vec4 color )
{
	const std::vector<vec3>& vertices = mesh->vertices();
	const std::vector<unsigned>& indices = mesh->render_indices();

	glUseProgram( m_program );
	glBindVertexArray( m_vao );

	// draw edges (half edge)

	// bind mesh data to buffers
	glBindBuffer( GL_ARRAY_BUFFER, m_vertex_buffer );
	glBufferData( GL_ARRAY_BUFFER, vertices.size() * sizeof( vec3 ), vertices.data(), GL_STATIC_DRAW );
	glEnableVertexAttribArray( 0 );
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vec3 ), ( void* )0 );

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_index_buffer );
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof( unsigned ), indices.data(), GL_STATIC_DRAW );

	int mvp_loc = glGetUniformLocation( m_program, "uniform_mvp" );
	int color_loc = glGetUniformLocation( m_program, "uniform_color" );

	// model to world transformation matrix
	mat4 translation	= glm::translate( mat4( 1.0f ), pos );
	mat4 scale			= glm::scale( mat4( 1.0f ), scl );
	mat4 rotation		= glm::toMat4( rot );

	mat4 mvp_mat = m_perspective * m_camera.world_to_cam() * translation * rotation * scale;


	glUniformMatrix4fv( mvp_loc, 1, GL_FALSE, &mvp_mat[0][0] );
	glUniform4fv( color_loc, 1, &color.x );

	glDrawElements( GL_LINES, static_cast< GLsizei >( indices.size() ), GL_UNSIGNED_INT, 0 );


	// draw faces

	const unsigned id = mesh->render_mesh();

	glBindBuffer( GL_ARRAY_BUFFER, m_vertex_buffer );
	glBufferData( GL_ARRAY_BUFFER, m_meshes[id].vertices.size() * sizeof( vec3 ), m_meshes[id].vertices.data(), GL_STATIC_DRAW );
	glEnableVertexAttribArray( 0 );
	glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, sizeof( vec3 ), ( void* )0 );

	glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_index_buffer );
	glBufferData( GL_ELEMENT_ARRAY_BUFFER, m_meshes[id].indices.size() * sizeof( ivec3 ), m_meshes[id].indices.data(), GL_STATIC_DRAW );

	vec4 face_color( 1.0f, 1.0f, 1.0f, 0.1f );
	glUniform4fv( color_loc, 1, &face_color.x );
	glDrawElements( GL_TRIANGLES, static_cast<GLsizei>( m_meshes[id].indices.size() * 3 ), GL_UNSIGNED_INT, 0 );

	glBindVertexArray( 0 );
}





/**
* @brief initialize glfw
*/
void Graphics::initialize_glfw() const
{
	// initialize glfw
	if ( !glfwInit() ) {
		std::cout << "error initializing glfw" << std::endl;
		std::abort();
	}
}

/**
* @brief create window context
* @return window
*/
GLFWwindow* Graphics::create_window()
{
	m_width = 1280;
	m_height = 720;

	// window and context
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 3 );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );
	glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE );
	glfwWindowHint( GLFW_VISIBLE, GLFW_TRUE );
	glfwWindowHint( GLFW_RESIZABLE, GLFW_FALSE );
	GLFWwindow* window = glfwCreateWindow( m_width, m_height, "CS550", nullptr, nullptr );

	if ( !window ) {
		std::cout << "error creating a window" << std::endl;
		std::abort();
	}
	glfwMakeContextCurrent( window );
	glfwSwapInterval( 1 );

	return window;
}

/**
* @brief initialize glad
*/
void Graphics::initialize_glad() const
{
	// Initialize GLAD
	if ( !gladLoadGLLoader( ( GLADloadproc )glfwGetProcAddress ) ) {
		std::cout << "error initializing glad" << std::endl;
		std::abort();
	}

	if ( !GLAD_GL_VERSION_3_3 ) {
		std::cout << "incorrect version of glad" << std::endl;
		std::abort();
	}
}

/**
* @brief create and compile a shader
* @param file_name		path to the file with the shader source code
* @param shader_type	type of shader its being created
* @return shader
*/
unsigned Graphics::compile_shader( const char * file_name, GLenum shader_type ) const
{
	// open the file
	std::ifstream file( file_name, std::ios::in );

	// check file is open
	if ( !file )
	{
		std::cout << "Unable to open file " << file_name << std::endl;
		std::abort();
		return 0u;
	}

	// copy the file content into a string
	std::string source( ( std::istreambuf_iterator<char>( file ) ), std::istreambuf_iterator<char>() );

	// colse file
	file.close();

	// create and compile the shader
	unsigned shader = glCreateShader( shader_type );
	const char* c_source = source.c_str();
	glShaderSource( shader, 1, &c_source, NULL );
	glCompileShader( shader );

	// check errors
	int error;
	char log[512];
	glGetShaderiv( shader, GL_COMPILE_STATUS, &error );

	if ( !error )
	{
		glGetShaderInfoLog( shader, 512, NULL, log );
		std::cout << "couldn't compile shader\n" << log << std::endl;
		std::abort();
	}

	return shader;
}

/**
* @brief create a shader program
* @param vertex_shader		path to the vertex shader source file
* @param fragment_shader	path to the fragment shader source file
* @return shader program
*/
unsigned Graphics::create_program( const char * vertex_shader, const char * fragment_shader ) const
{
	// create and compile the shaders
	unsigned vert = compile_shader( vertex_shader, GL_VERTEX_SHADER );
	unsigned frag = compile_shader( fragment_shader, GL_FRAGMENT_SHADER );

	// create the program, attatch the shaders and link it
	unsigned program = glCreateProgram();
	glAttachShader( program, vert );
	glAttachShader( program, frag );
	glLinkProgram( program );

	// check errors
	int error;
	char log[512];
	glGetProgramiv( program, GL_LINK_STATUS, &error );

	if ( !error )
	{
		glGetProgramInfoLog( program, 512, NULL, log );
		std::cout << "couldn't link program\n" << log << std::endl;
		std::abort();
	}

	// delete the shaders (we don't need them anymore)
	glDeleteShader( vert );
	glDeleteShader( frag );

	return program;
}

/**
* @brief load the meshes from the obj files
*/
void Graphics::load_meshes()
{
	// load OBJs
	m_meshes.push_back( load_obj( "../resources/meshes/cube.obj" ) );
	m_meshes.push_back( load_obj( "../resources/meshes/cylinder.obj" ) );
	m_meshes.push_back( load_obj( "../resources/meshes/icosahedron.obj" ) );
	m_meshes.push_back( load_obj( "../resources/meshes/octohedron.obj" ) );
	m_meshes.push_back( load_obj( "../resources/meshes/sphere.obj" ) );
}
