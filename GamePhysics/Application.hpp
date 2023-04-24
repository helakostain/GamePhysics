#pragma once
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
#include "Scene.hpp"
#include "Callbacks.hpp"

class Application
{
private:
	GLFWwindow* window;
	static Application* instance;

	void StartGLFW(); //initialize GLFW
	void StartGLEW(); //initialize GLEW
	void CreateNewWindow();
	void VersionInfo(); //Writes all Versions to console
public:
	Application();
	~Application();

	int height;
	int width;

	static Application& getInstance();

	void Run();
};