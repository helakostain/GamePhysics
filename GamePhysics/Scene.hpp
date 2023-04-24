#pragma once
#include <chrono>
#include <optional>
#include <iostream>
#include <functional>
#include <vector>
#include <ctype.h>
#include <algorithm>
#include "DrawableObject.hpp"
#include "Camera.hpp"
#include "Shader.hpp"
#include "Callbacks.hpp"
#include "Observer.hpp"
#include "Light.hpp"
#include "Skybox.hpp"
#include "Texture.hpp"
#include "PxPhysicsAPI.h"
#include "PsTime.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include <GLFW/glfw3.h> // DO NOT MOVE UP!!!!!!

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
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
	physx::PxCooking* gCooking = NULL;
	physx::PxDefaultCpuDispatcher* gDispatcher = NULL;
	physx::PxScene* gScene = NULL;
	physx::PxMaterial* gMaterial = NULL;
	physx::PxPvd* gPvd = NULL;
	physx::PxControllerManager* gControllerManager = NULL;
	physx::PxController* gController = NULL;
	physx::PxCudaContextManager* gCudaContextManager = NULL;

	std::unordered_map<physx::PxActor*, int> actorID;
	int num_balls = 0;
	bool ball_exist = false;
	bool cudaON = false;  //true for cuda acceleration, false for CPU computation

	void initPhysics();
	void createTriangleMeshes(int i, int j);
	void createCharacter(int i, int j, physx::PxRigidDynamic* actor, physx::PxShape* shape);
	void createConvexMeshes(int i, int j);
	void createStaticActor(int i, int j);
	void setupCommonCookingParams(physx::PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData);

	void stepPhysics(double delta);
	void cleanupPhysics();

	void applyPhysXTransform(const float delta, const physx::PxVec3);
	physx::PxRigidDynamic* shootBall(const physx::PxTransform& t, const physx::PxGeometry& geometry, const physx::PxVec3& velocity = physx::PxVec3(0));

	void createForest();
public:
	Scene(GLFWwindow* window);

	std::vector<DrawableObject> drawable_object;

	void Run();

	void notify(EventType eventType, void* object);
};