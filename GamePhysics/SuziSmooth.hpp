#pragma once
#include <GL/glew.h>
#include <glm/mat4x4.hpp>
#include "Models/suzi_smooth.h"
#include "Models.hpp"
#include <iterator>
#include <vector>

class SuziSmooth : public Models
{
public:
	SuziSmooth();
	void Draw() override;
	void Init() override;
	void Bind() override;
};