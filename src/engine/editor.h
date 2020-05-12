/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: editor.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/28/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#include "scene.h"
#include <string>

class Editor
{
private:
	Editor() {}

public:
	static Editor& get_instance();

	void initialize	();
	void update		();
	void exit		();

	void render		();

private:
	void reload();

private:
	Scene m_scene;
	std::vector<std::string> m_scene_files;
	int m_selected_scene;
};
