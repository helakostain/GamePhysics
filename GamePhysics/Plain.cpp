#include "Plain.hpp"
#include "Models/plain.h"

Plain::Plain() : Models()
{
	this->points = plain;
	this->size_points = sizeof(plain);

	this->valuesInRow = 8;
	this->skip = 3;
	this->values = 3;
	this->VAO = 0;
	this->VBO = 0;
}

void Plain::Draw()
{
	glDrawArrays(GL_TRIANGLES, 0, 6);
}

void Plain::Init()
{
	glGenBuffers(1, &this->VBO);

	glBindBuffer(GL_ARRAY_BUFFER, this->VBO);
	glBufferData(GL_ARRAY_BUFFER, this->size_points * sizeof(float), this->points, GL_STATIC_DRAW);

	glGenVertexArrays(1, &this->VAO);
	glBindVertexArray(this->VAO);

	glBindBuffer(GL_ARRAY_BUFFER, this->VBO);

	glVertexAttribPointer(0, values, GL_FLOAT, GL_FALSE, valuesInRow * sizeof(float), NULL);
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(2, values, GL_FLOAT, GL_FALSE, valuesInRow * sizeof(float), (void*)(skip * sizeof(float)));
	glEnableVertexAttribArray(2);

	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, valuesInRow * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(1);
}

void Plain::Bind()
{
	glBindVertexArray(this->VAO);
}