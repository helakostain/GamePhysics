#include "Models.hpp"
#include "ShaderInstances.hpp"

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

void Models::setShader(Shader* shader)
{
	this->shader = shader;
}

void Models::addMesh(Mesh&& mesh)
{
	meshes.emplace_back(std::move(mesh));
	transformations.push_back(new Transformation());
	shaders.push_back(new Shader(ShaderInstances::phong()));
}

void Models::draw(uint32_t id, int i) const
{
	//int i = 0;
	//for (const Mesh& mesh : meshes) {
	meshes[i].bindAndDraw(id, shaders[i]);
		//mesh.bindAndDraw(id, shaders[i]);
		//i++;
	//}
}

void Models::applyPhysxTransf(glm::mat4 a, int actorID)
{
	//this->transformations[actorID].translate(a);
	//this->transformations[actorID]->setPosition(a[3]);
	this->currPosition = a[3];
	this->transformations[actorID]->setMatrix(a);
	//this->shaders[actorID]->updatePosition(a);
}

int Models::get_size_points()
{
	return this->size_points;
}

Transformation* Models::getTransformation(int i)
{
	return this->transformations[i];
}

void Models::DoTransformations(const double delta, int i)
{
	//for (int i = 0; i < meshes.size(); i++)
	//{
		this->transformations[i]->Update(delta);
	//}
}

void Models::Pos_scale(float a)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->scale(a);
	}
}

void Models::setFy(Direction dir)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->applyFy(dir);
	}
}
void Models::setFx(Direction dir)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->applyFx(dir);
	}
}
void Models::setRot(Rotation r)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->setRotation(r);
	}
}
void Models::setGrow(Growth g)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->setGrowth(g);
	}
}

void Models::Pos_mov(glm::vec3 a)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->translate(a);
	}
	this->currPosition = a;
}

void Models::rotate(float degree, glm::vec3 axis)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->rotate(degree, axis);
	}
}

void Models::setPos(glm::vec3 position)
{
	for (int i = 0; i < meshes.size(); i++)
	{
		this->transformations[i]->setPosition(position);
	}
}