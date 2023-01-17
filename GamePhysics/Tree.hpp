#pragma once
#include <GL/glew.h>
#include <glm/mat4x4.hpp>
#include "Models/tree.h"
#include "Models.hpp"
#include <iterator>
#include <vector>

class Tree : public Models
{
public:
	Tree();
	void Draw() override;
	void Init() override;
	void Bind() override;
};