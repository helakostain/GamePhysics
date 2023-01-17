#pragma once
#include <GL/glew.h>
#include <glm/mat4x4.hpp>
#include "Models.hpp"
#include <iterator>
#include <vector>

class Bushes : public Models
{
public:
	Bushes();
	void Draw() override;
	void Init() override;
	void Bind() override;
};