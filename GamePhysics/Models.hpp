#pragma once
#include <GL/glew.h>
#include <glm/vec3.hpp> 
#include <glm/vec4.hpp> 
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <unordered_map>
#include <string>

#include "Camera.hpp"
#include "Mesh.hpp"
#include "ModelsLoader.hpp"
#include "Shader.hpp"

class ModelsLoader;

class Models
{
private:
	friend ModelsLoader;

	void GenerateVBO();
	void GenerateVAO(int valuesInRow, int skip, int values);
protected:
	const float* points;
	int size_points;
	int valuesInRow;
	int skip;
	int values;

	GLuint VBO;
	GLuint VAO;
public:
	Models();
	Models(const float in_points[], int size_points);

	virtual void Init();
	virtual void Init(int valuesInRow, int skip, int values);
	virtual void Bind();
	virtual void Draw();

	void setIds(GLuint vID, GLuint fID);

	std::vector<Mesh> meshes;
	std::vector<Material> materials;
	std::string directory;

	// 3D models
	void addMesh(Mesh&& mesh);
	void draw(uint32_t id, Shader* shader) const;
};