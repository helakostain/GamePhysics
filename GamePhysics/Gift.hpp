#pragma once
#include <GL/glew.h>
#include <glm/mat4x4.hpp>
#include "Models/gift.h"
#include "Models.hpp"
#include <iterator>
#include <vector>

class Gift : public Models
{
public:
	Gift();
	void Draw() override;
	void Init() override;
	void Bind() override;
};