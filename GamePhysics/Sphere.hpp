#pragma once
#include <GL/glew.h>
#include <glm/mat4x4.hpp>
#include "Models/sphere.h"
#include "Models.hpp"
#include <iterator>
#include <vector>

class Sphere : public Models
{
public:
	Sphere();
	void Draw() override;
	void Init() override;
	void Bind() override;
};