#version 330 core

in vec3 attr_position;

void main()
{
	gl_Position = vec4(attr_position, 1.0f);
}