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
/*
Models::Models(Models&& old) noexcept :
	transformations(std::move(old.transformations)), meshes(std::move(old.meshes)), materials(std::move(old.materials)), directory(old.directory), shaders(std::move(old.shaders))
{
}*/

Models::Models(Models& old) noexcept
{
	//this->transformations.push_back(old.transformations[0]);
	for (int i = 0; i < old.transformations.size(); i++)
	{
		//this->transformations.push_back(std::move(old.transformations[i]));
		this->transformations.push_back(new Transformation());
	}
	//std::copy(old.transformations.begin(), old.transformations.end(), this->transformations.begin());
	//this->meshes = old.meshes; //TODO: crashuje to tu, asi se tomu nelibi ze ty meshe neexistuji pri kompilaci????
	for (int i = 0; i < old.meshes.size(); i++)
	{
		//Mesh mesh(old.meshes[i]);
		//this->meshes.push_back(old.meshes[i]);
		const Material& mat = { old.meshes[i].material.diffuse, old.meshes[i].material.specular, old.meshes[i].material.ambient, old.meshes[i].material.diffuseMap, old.meshes[i].material.specularMap, old.meshes[i].material.heightMap, old.meshes[i].material.shininess};
		std::vector<Vertex> vertices;
		for (int j = 0; j < old.meshes[i].vertices.size(); j++)
		{
			vertices.push_back(old.meshes[i].vertices[j]);
		}
		std::vector<uint32_t> indices;
		for (int j = 0; j < old.meshes[i].indices.size(); j++)
		{
			indices.push_back(old.meshes[i].indices[j]);
		}
		Mesh mesh(vertices, indices, mat);
		this->meshes.emplace_back(std::move(mesh));
	}
	//std::copy(old.meshes.begin(), old.meshes.end(), this->meshes.begin());
	//this->materials = old.materials;
	for (int i = 0; i < old.materials.size(); i++)
	{
		this->materials.push_back(old.materials[i]);
	}
	//std::copy(old.materials.begin(), old.materials.end(), this->materials.begin());
	this->directory = old.directory;
	//this->shaders = old.shaders;
	for (int i = 0; i < old.shaders.size(); i++)
	{
		this->shaders.push_back(old.shaders[i]);
	}
	//std::copy(old.shaders.begin(), old.shaders.end(), this->shaders.begin());
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
	if (isBall)
	{
		this->transformations[0]->setMatrix(a);
	}
	else
	{
		this->transformations[actorID]->setMatrix(a);
	}
	
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