#pragma once
#include <GL/glew.h>
#include <glm/mat4x4.hpp>
#include "Models/suzi_flat.h"
#include "Models.hpp"
#include <iterator>
#include <vector>

class SuziFlat : public Models
{
public:
	SuziFlat();
	void Draw() override;
	void Init() override;
	void Bind() override;
};