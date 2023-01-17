#include "SuziSmooth.hpp"

SuziSmooth::SuziSmooth() : Models()
{
	this->points = suziSmooth;
	this->size_points = sizeof(suziSmooth);

	this->valuesInRow = 6;
	this->skip = 3;
	this->values = 3;
	this->VAO = 0;
	this->VBO = 0;
}

void SuziSmooth::Draw()
{
	glDrawArrays(GL_TRIANGLES, 0, 2904);
}

void SuziSmooth::Init()
{
	glGenBuffers(1, &this->VBO);

	glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
	glBufferData(GL_ARRAY_BUFFER, this->size_points * sizeof(float), this->points, GL_STATIC_DRAW);

	glGenVertexArrays(1, &this->VAO);
	glBindVertexArray(this->VAO);

	glBindBuffer(GL_ARRAY_BUFFER, this->VBO);

	glVertexAttribPointer(0, values, GL_FLOAT, GL_FALSE, valuesInRow * sizeof(float), NULL);
	glEnableVertexAttribArray(0);

	if (skip != 0) {
		glVertexAttribPointer(1, values, GL_FLOAT, GL_FALSE, valuesInRow * sizeof(float), (void*)(skip * sizeof(float)));
		glEnableVertexAttribArray(1);
	}
}

void SuziSmooth::Bind()
{
	glBindVertexArray(this->VAO);
}