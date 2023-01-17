#include "Application.hpp"

Application* Application::instance = nullptr;

void Application::StartGLFW()
{
	if (!glfwInit()) {
		fprintf(stderr, "ERROR: could not start GLFW3\n");
		exit(EXIT_FAILURE);
	}
}

void Application::StartGLEW()
{
	// start GLEW extension handler
	glewExperimental = GL_TRUE;
	glewInit();
}

void Application::CreateWindow()
{
	window = glfwCreateWindow(width, height, "Game physics", NULL, NULL);
	if (!window) {
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
}

void Application::VersionInfo()
{
	// get version info
	printf("OpenGL Version: %s\n", glGetString(GL_VERSION));
	printf("Using GLEW %s\n", glewGetString(GLEW_VERSION));
	printf("Vendor %s\n", glGetString(GL_VENDOR));
	printf("Renderer %s\n", glGetString(GL_RENDERER));
	printf("GLSL %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));
	int major, minor, revision;
	glfwGetVersion(&major, &minor, &revision);
	printf("Using GLFW %i.%i.%i\n", major, minor, revision);
}

Application::Application()
{
	this->height = 900;
	this->width = 1600; // HD+ resolution as default
	StartGLFW();
	CreateWindow();
}

Application::~Application()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

Application& Application::getInstance()
{
	if (not instance)
	{
		instance = new Application();
	}
	return *instance;
}

void Application::Run()
{
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);
	StartGLEW();
	VersionInfo();
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	float ratio = width / (float)height;
	glViewport(0, 0, width, height);
	Scene scene(window);
	scene.Run();
}