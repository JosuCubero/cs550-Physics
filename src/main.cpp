/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: main.cpp
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/20/2020
----------------------------------------------------------------------------------------------------------*/
#include "graphics.h"
#include "physics.h"
#include "editor.h"

int main(int argc, char** argv)
{
	Graphics&	graphics	= Graphics::get_instance();
	Physics&	physics		= Physics::get_instance();
	Editor&		editor		= Editor::get_instance();

	// initialize
	graphics.initialize();
	physics.initialize();
	editor.initialize();

	float time = static_cast<float>( glfwGetTime() );
	float last_time = time;

	// Render loop
	while (!glfwWindowShouldClose(graphics.window())){

		time = static_cast< float >( glfwGetTime() );

		glfwPollEvents();
		
		editor.update();
		// physics update
		physics.update( time - last_time );
		// graphics update
		graphics.update();


		last_time = time;
	}

	// Destroy
	physics.exit();
	graphics.exit();
	editor.exit();
	
	return 0;
}
