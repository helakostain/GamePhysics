#pragma once
#include <chrono>
#include <optional>
#include <iostream>
#include <functional>
#include <vector>
#include <ctype.h>
#include "DrawableObject.hpp"
#include "Camera.hpp"
#include "Shader.hpp"
#include "Callbacks.hpp"
#include "Observer.hpp"
#include "Light.hpp"
#include "Skybox.hpp"
#include "Texture.hpp"
#include "PxPhysicsAPI.h"
#include <GLFW/glfw3.h> // DO NOT MOVE UP!!!!!!

class Scene : public Observer
{
private:
	typedef decltype(std::chrono::high_resolution_clock::now()) TimePoint;
	typedef std::chrono::duration<double, std::ratio<1>> Second;

	Camera* camera;
	GLFWwindow* window;
	Mouse& mouse = Mouse::instance();
	std::vector<std::shared_ptr<ColoredLight>> lights;
	AmbientLight ambientLight;
	std::shared_ptr<Skybox> skybox;

	void Loop();

	size_t lightCount() const;
	void emplaceAmbientLight(glm::vec3 color);
	void initAndEmplace(std::shared_ptr<ColoredLight>& light);
	void emplaceLight(const glm::vec3 color, const glm::vec3 pos, const gl::Light type);
	void emplaceLight(glm::vec3 color, glm::vec3 pos, glm::vec3 dir, float cutoff);
	std::shared_ptr<ColoredLight> createLight(glm::vec3 color, glm::vec3 data, gl::Light type);
	void applyLights() const;
	void placeModel(const int mouseX, const int mouseY);
	void setShaderCount() const;

	void onButtonPress(const MouseData& mouseData);

	//PhysX
	physx::PxDefaultAllocator gAllocator;
	physx::PxDefaultErrorCallback gErrorCallback;

	physx::PxFoundation* gFoundation = NULL;
	physx::PxPhysics* gPhysics = NULL;

	physx::PxDefaultCpuDispatcher* gDispatcher = NULL;
	physx::PxScene* gScene = NULL;

	physx::PxMaterial* gMaterial = NULL;

	physx::PxPvd* gPvd = NULL;

	physx::PxCooking* mCooking = NULL;

	physx::PxReal stackZ = 10.0f;
	physx::PxRigidDynamic* createDynamic(const physx::PxTransform& t, const physx::PxGeometry& geometry, const physx::PxVec3& velocity = physx::PxVec3(0));
	void createStack(const physx::PxTransform& t, physx::PxU32 size, physx::PxReal halfExtent);
	void initPhysics(bool interactive);
	void stepPhysics(bool /*interactive*/);
	void cleanupPhysics(bool /*interactive*/);
	void keyPress(unsigned char key, const physx::PxTransform& camera);
	
	bool toPhysxActor(int i);
	void applyPhysXTransform();
public:
	Scene(GLFWwindow* window);

	std::vector<DrawableObject> drawable_object;

	void Run();

	void notify(EventType eventType, void* object);
};