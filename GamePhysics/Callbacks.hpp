#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <optional>
#include <memory>

#include "Application.hpp"
#include "DrawableObject.hpp"
#include "Camera.hpp"
#include "Mouse.hpp"
#include "Scene.hpp"
#include "Light.hpp"
//#include "PxPhysicsAPI.h"

class Callbacks
{
private:
	static double clickX;
	static double clickY;
	static std::vector<DrawableObject> drawableObj;
	static Camera* camera;
	static shared_ptr<ColoredLight> flashlight;
	//static physx::PxController* controller;
	//static physx::PxScene* scene;

	static void error_callback(int error, const char* description);
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void cursor_callback(GLFWwindow* window, double x, double y);
	static void button_callback(GLFWwindow* window, int button, int action, int mode);

	static void window_focus_callback(GLFWwindow* window, int focused);
	static void window_iconify_callback(GLFWwindow* window, int iconified);
	static void window_size_callback(GLFWwindow* window, int width, int height);
public:
	Callbacks();
	~Callbacks();

	static void Init(GLFWwindow* window, std::vector<DrawableObject>& dO, Camera* camera, shared_ptr<ColoredLight> new_flashlight);
	static void Init(GLFWwindow* window, std::vector<DrawableObject>& dO, Camera* camera);
	static void setObject(int id);
	static void updateObjects(std::vector<DrawableObject>& dObjects);
	//static void updateCharacter(physx::PxController* cont);
};