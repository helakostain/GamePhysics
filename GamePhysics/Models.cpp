#include "Models.hpp"

void Models::GenerateVBO()
{
	//vertex buffer object (VBO)
	glGenBuffers(1, &VBO); // generate the VBO
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, size_points * sizeof(float), points, GL_STATIC_DRAW);
}

void Models::GenerateVAO(int valuesInRow, int skip, int values)
{
	//Vertex Array Object (VAO)
	glGenVertexArrays(1, &VAO); //generate the VAO
	glBindVertexArray(VAO); //bind the VAO
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glVertexAttribPointer(0, values, GL_FLOAT, GL_FALSE, valuesInRow * sizeof(float), NULL); //points
	glEnableVertexAttribArray(0); //enable vertex attributes
	if (skip != 0) {
		glVertexAttribPointer(1, values, GL_FLOAT, GL_FALSE, valuesInRow * sizeof(float), (void*)(skip * sizeof(float))); //colours or lightning
		glEnableVertexAttribArray(1); //enable vertex attributes
	}
}

Models::Models()
{
}

Models::Models(const float in_points[], int size_points)
{
	this->VAO = 0;
	this->VBO = 0;
	this->points = in_points;
	this->size_points = size_points;
}

void Models::Init()
{
}

void Models::Init(int valuesInRow, int skip, int values)
{
	GenerateVBO();
	GenerateVAO(valuesInRow, skip, values);
}

void Models::Bind()
{
	glBindVertexArray(VAO);
}

void Models::Draw()
{
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Models::setIds(GLuint vID, GLuint fID)
{
	this->VAO = vID;
	this->VBO = fID;
}

void Models::addMesh(Mesh&& mesh)
{
	meshes.emplace_back(std::move(mesh));
}

void Models::draw(uint32_t id, Shader* shader) const
{
	for (const Mesh& mesh : meshes) {
		mesh.bindAndDraw(id, shader);
	}
}