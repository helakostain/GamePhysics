#pragma once
#include <vector>
#include <optional>
#include "Models.hpp"
#include "Shader.hpp"
#include "Transformation.hpp"
#include "Texture.hpp"
#include "MovementCalculator.hpp"

class DrawableObject
{
private:
	Models* models;
	//Shader* shaders;
	//Transformation* transformations;
	shared_ptr<Texture> texture;
	shared_ptr<MovementCalculator> movementCalculator = nullptr;

	const char* vertex_shader;
	const char* fragment_shader;

	bool isObject;
	int id;
	int size_points;

	int actorType; //0 = Static; 1 = Dynamic, 2 = Kinematic
public:
	DrawableObject();
	DrawableObject(Models* model, const char* vertex_path, const char* fragment_path, int size);
	DrawableObject(Models* model, Shader& shader, int size);
	DrawableObject(Models* model, Shader& shader, shared_ptr<MovementCalculator> movement, int size);
	DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, int size);
	DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, int size, bool object, int actorType);
	DrawableObject(Models* model, Shader& shader, shared_ptr<Texture> texture, shared_ptr<MovementCalculator> movement, int size, bool object);

	//void DoTransformations(const double delta);
	void sendShaderMatrix(int i);
	bool SetUp();
	/*void Pos_scale(float a);
	void setFy(Direction dir);
	void setFx(Direction dir);
	void setRot(Rotation r);
	void setGrow(Growth g);
	void Pos_mov(glm::vec3 a);*/
	void Draw();
	void updateObject(const float delta);
	void applyTexture(std::shared_ptr<Texture> texture);
	//void rotate(float degree, glm::vec3 axis);
	void updateMovement(double delta);
	int getId();
	int getSizePoints();
	Models* getModel();
	int getActorType();
	//Transformation* getTransformation();
	//glm::vec3 currPosition = { 0,0,0 };
};