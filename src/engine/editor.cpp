/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: editor.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/28/2020
----------------------------------------------------------------------------------------------------------*/
#include "editor.h"

#include "graphics.h"

#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include <filesystem>

/**
* @brief get instance of the singleton
* @return instance
*/
Editor& Editor::get_instance()
{
	static Editor instance{};
	return instance;
}

/**
* @brief initialize editor
*/
void Editor::initialize()
{
	// initialize ImGui
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	( void )io;

	ImGui_ImplGlfw_InitForOpenGL( Graphics::get_instance().window(), true );
	ImGui_ImplOpenGL3_Init( "#version 130" );

	// get all scene files
	for ( const auto& entry : std::filesystem::directory_iterator( "../resources/scenes/" ) )
		m_scene_files.push_back( entry.path().u8string() );

	// load first scene
	m_selected_scene = 0;
	m_scene.load_scene( m_scene_files[m_selected_scene].c_str() );
}

/**
* @brief update editor
*/
void Editor::update()
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// print debug
	ImGui::Begin( "debug", nullptr, ImGuiWindowFlags_AlwaysAutoResize );
	{
		ImGui::Text( "fps : %d", static_cast<int>( glm::round( ImGui::GetIO().Framerate ) ) );

		ImGui::SliderInt( "Scene", &m_selected_scene, 0, m_scene_files.size() - 1 );

		if ( ImGui::Button( "Reload" ) )
			reload();
	}
	ImGui::End();

	// create a ray from the camera
	Ray ray;
	ray.origin = Graphics::get_instance().camera().position();
	ray.direction = normalize( Graphics::get_instance().mouse_pos() - ray.origin );


	// if mouse click
	if ( glfwGetMouseButton( Graphics::get_instance().window(), GLFW_MOUSE_BUTTON_1 ) && ImGui::IsAnyWindowFocused() == false )
	{
		// raycast the scene
		Physics::get_instance().push_object( ray );
	}

	Physics::get_instance().show_in_editor();
}

/**
* @brief exit editor
*/
void Editor::exit()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

/**
* @brief render editor
*/
void Editor::render()
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
}

/**
* @reload systems and scene
*/
void Editor::reload()
{
	Physics::get_instance().clear();
	m_scene.load_scene( m_scene_files[m_selected_scene].c_str() );
}
