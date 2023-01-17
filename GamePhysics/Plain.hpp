#pragma once
#include <GL/glew.h>
#include <glm/mat4x4.hpp>
#include "Models.hpp"
#include <iterator>
#include <vector>

class Plain : public Models
{
public:
	Plain();
	void Draw() override;
	void Init() override;
	void Bind() override;
};