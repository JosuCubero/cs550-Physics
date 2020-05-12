/* ---------------------------------------------------------------------------------------------------------
Copyright (C) 2020 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior written
consent of DigiPen Institute of Technology is prohibited.

File Name: math_utils.h
Author: Josu Cubero Ruiz de Gopegui, josu.cubero, 540001316
Creation date: 01/07/2020
----------------------------------------------------------------------------------------------------------*/
#pragma once

#define GLM_FORCE_INLINE
#define GLM_FORCE_NO_CTOR_INIT
#define GLM_FORCE_EXPLICIT_CTOR
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/compatibility.hpp>
#include <glm/common.hpp>
#include <glm/gtc/type_ptr.hpp>

typedef glm::vec2	vec2;
typedef glm::vec3	vec3;
typedef glm::vec4	vec4;
typedef glm::mat3	mat3;
typedef glm::mat4	mat4;
typedef glm::quat	quat;
typedef glm::ivec3	ivec3;
typedef glm::ivec4	ivec4;
