#include "Scene.hpp"
#include "Sphere.hpp"
#include "Gift.hpp"
#include "Plain.hpp"
#include "SuziFlat.hpp"
#include "SuziSmooth.hpp"
#include "Tree.hpp"
#include "Bushes.hpp"
#include "ShaderInstances.hpp"
#include "Skybox.hpp"
#include "MoveCircle.hpp"
#include "MoveLine.hpp"
#include "MoveBezier.hpp"

#define draw_objects 150

void Scene::Loop()
{
	TimePoint lastTime = std::chrono::high_resolution_clock::now(); //current time
	skybox->Draw();

	glClear(GL_DEPTH_BUFFER_BIT); //clear window content
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_STENCIL_TEST);
	glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
	glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

	for (int i = 0; i < this->drawable_object.size(); i++) //first draw of scene
	{
		this->drawable_object[i].updateObject(0.f);
		camera->update(1.f);
	}
	glfwPollEvents(); // update other events like input handling
	glfwSwapBuffers(window); // put the stuff weve been drawing onto the display
	camera->apply(); //applying camera
	ambientLight.apply();
	applyLights();

	while (!glfwWindowShouldClose(window)) {  //main while loop for constant rendering of scene
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT); // clear color and depth buffer
		
		skybox->Draw();

		glClear(GL_DEPTH_BUFFER_BIT); //clear window content
		const TimePoint now = std::chrono::high_resolution_clock::now(); //new current time
		const float delta = std::chrono::duration_cast<Second>(now - lastTime).count(); //change of before and now time

		ambientLight.apply();
		applyLights();

		this->drawable_object[3].Pos_mov(glm::vec3(2.f, 2.f, 6.f));
		this->drawable_object[3].rotate(0.01f, glm::vec3(1.f, 0.f, 0.f));
		this->drawable_object[3].Pos_mov(glm::vec3(-2.f, -2.f, -6.f));

		for (int i = 0; i < this->drawable_object.size(); i++) //apply for all draw objects
		{
			this->drawable_object[i].updateObject(delta);
			lastTime = now;
		}

		camera->update(delta);
		lights[1]->update(camera->direction(), camera->position());

		glfwPollEvents(); // update other events like input handling
		glfwSwapBuffers(window); // put the stuff weve been drawing onto the display
	}
}

size_t Scene::lightCount() const
{
	return lights.size();
}

void Scene::emplaceAmbientLight(glm::vec3 color)
{
	ambientLight = AmbientLight{ color };
	ambientLight.registerObserver(ShaderInstances::constant());
	ambientLight.registerObserver(ShaderInstances::phong());
	ambientLight.registerObserver(ShaderInstances::terrain());
	ambientLight.registerObserver(ShaderInstances::phong_no_textures());
	ambientLight.registerObserver(ShaderInstances::phong_norm());
}

void Scene::initAndEmplace(std::shared_ptr<ColoredLight>& light)
{
	light->registerObserver(ShaderInstances::constant());
	light->registerObserver(ShaderInstances::phong());
	light->registerObserver(ShaderInstances::terrain());
	light->registerObserver(ShaderInstances::phong_no_textures());
	light->registerObserver(ShaderInstances::phong_norm());
	light->lightIndex = this->lights.size();
	this->lights.emplace_back(light);
}

void Scene::emplaceLight(const glm::vec3 color, const glm::vec3 pos, const gl::Light type)
{
	std::shared_ptr<ColoredLight> light = createLight(color, pos, type);
	this->drawable_object.emplace_back(new Sphere(), ShaderInstances::constant(), drawable_object.size());
	this->drawable_object.back().Pos_mov(pos);
	this->drawable_object.back().Pos_mov(glm::vec3(0.f, 0.f, (0.1 * (pos.z / abs(pos.z)))));
	this->drawable_object.back().Pos_scale(0.25);
	initAndEmplace(light);
	applyLights();
}

void Scene::emplaceLight(glm::vec3 color, glm::vec3 pos, glm::vec3 dir, float cutoff)
{
	std::shared_ptr<ColoredLight> light = std::make_shared<Spotlight>(color, pos, dir, cutoff);
	this->drawable_object.emplace_back(new Sphere(), ShaderInstances::constant(), drawable_object.size());
	this->drawable_object.back().Pos_mov(pos);
	this->drawable_object.back().Pos_mov(glm::vec3(0.f, 0.f, 0.1f));
	this->drawable_object.back().Pos_scale(0.25);
	initAndEmplace(light);
	applyLights();
}

std::shared_ptr<ColoredLight> Scene::createLight(glm::vec3 color, glm::vec3 data, gl::Light type)
{
	if (type == gl::Light::Point)
	{
		return std::make_shared<PositionedLight>(color, data);
	}
	else if (type == gl::Light::Directional) {
		return std::make_shared<DirectionalLight>(color, data);
	}
	throw std::runtime_error("Unsupported light type");
}

void Scene::applyLights() const
{
	setShaderCount();
	for (const auto& light : lights)
	{
		light->apply();
	}
}

void Scene::placeModel(const int mouseX, const int mouseY)
{
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	GLfloat depth;
	GLint index;
	const GLint x = mouseX;
	const GLint y = height - mouseY;

	glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
	glReadPixels(x, y, 1, 1, GL_STENCIL_INDEX, GL_INT, &index);
	printf("Object id: %d\n", index);
	if (index != 0) {
		glm::vec3 screenX = glm::vec3{ x, y, depth };
		glm::vec4 viewport{ 0, 0, width, height };
		auto pos = glm::unProject(screenX, camera->view(), camera->project(), viewport);
		pos.y = 0;
		this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree", "Textures/tree.png"), drawable_object.size(), true));
		drawable_object.back().Pos_mov(pos);
		drawable_object.back().Pos_scale(0.3);
		Callbacks::updateObjects(std::ref(drawable_object));
	}
}

void Scene::setShaderCount() const
{
	ShaderInstances::phong().passUniformLocation("lightCount", (int32_t)lights.size());
	ShaderInstances::phong_no_textures().passUniformLocation("lightCount", (int32_t)lights.size());
	ShaderInstances::terrain().passUniformLocation("lightCount", (int32_t)lights.size());
	ShaderInstances::phong_norm().passUniformLocation("lightCount", (int32_t)lights.size());
}

void Scene::onButtonPress(const MouseData& mouseData) {
	if (mouseData.mbPressed()) {
		placeModel(mouseData.x, mouseData.y);
	}
	else if (mouseData.rbPressed()) {
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);

		const GLint x = mouseData.x;
		const GLint y = height - mouseData.y;
		GLint index;
		glReadPixels(x, y, 1, 1, GL_STENCIL_INDEX, GL_INT, &index);
		printf("Object id: %d\n", index);
		Callbacks::setObject(index);
	}
}


Scene::Scene(GLFWwindow* in_window)
{
	this->window = in_window;

	static std::vector<std::string> cubemapTextures{
		/*"Textures/cubemap/posx.jpg",
		"Textures/cubemap/negx.jpg",
		"Textures/cubemap/posy.jpg",
		"Textures/cubemap/negy.jpg",
		"Textures/cubemap/posz.jpg",
		"Textures/cubemap/negz.jpg"*/
		"Textures/nightskybox/posx.jpg",
		"Textures/nightskybox/negx.jpg",
		"Textures/nightskybox/posy.jpg",
		"Textures/nightskybox/negy.jpg",
		"Textures/nightskybox/posz.jpg",
		"Textures/nightskybox/negz.jpg"
	};

	srand(time(NULL));
	this->skybox = std::make_shared<Skybox>(TextureManager::cubeMap("skybox", cubemapTextures));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("teren"), ShaderInstances::phong(), TextureManager::getOrEmplace("teren", "Textures/grass.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(0, 0.f, 10));
	this->drawable_object.emplace_back(DrawableObject(new SuziFlat(), ShaderInstances::phong_no_textures(), drawable_object.size()));
	this->drawable_object.back().Pos_mov(glm::vec3(20, 2.f, -5));
	this->drawable_object.emplace_back(DrawableObject(new SuziFlat(), ShaderInstances::phong_no_textures(), drawable_object.size()));
	this->drawable_object.back().Pos_mov(glm::vec3(2, 2.f, 8));
	this->drawable_object.emplace_back(DrawableObject(new SuziSmooth(), ShaderInstances::phong_no_textures(), drawable_object.size()));
	this->drawable_object.back().Pos_mov(glm::vec3(-15.f, 0.f, 6.f));
	this->drawable_object.emplace_back(DrawableObject(new SuziSmooth(), ShaderInstances::phong_no_textures(), std::make_shared<MovementCalculator>(
		std::make_shared<MoveBezier>(
			glm::vec3{ 40.f, 10.f, 25.f },
			glm::vec3{ -40.f, 10.f, 25.f },
			glm::vec3{ -40.f, 10.f, -25.f },
			glm::vec3{ 40.f, 10.f, -25.f }),
		glm::vec3{ 0.0, 0.0, 0.0 },
		10.f
		), drawable_object.size()));
	this->drawable_object.back().Pos_mov(glm::vec3(4, 2.f, 2));
	this->drawable_object.emplace_back(DrawableObject(new Gift(), ShaderInstances::phong_no_textures(), drawable_object.size()));
	this->drawable_object.back().Pos_mov(glm::vec3(4, 0.f, 2));
	/*this->drawable_object.emplace_back(DrawableObject(new Sphere(), ShaderInstances::phong_no_textures(), drawable_object.size()));
	this->drawable_object.back().Pos_mov(glm::vec3(-5, 2.f, 10));*/
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere"), ShaderInstances::phong_norm(), TextureManager::getOrEmplace("sphere", "Textures/sphere_albedo.png", "Textures/sphere_normal.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(-5, 2.f, 10));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("model"), ShaderInstances::phong(), TextureManager::getOrEmplace("house", "Textures/house.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(-15, 0.f, 10));
	this->drawable_object.back().rotate(45, glm::vec3(0.0f, 1.f, 0.f));
	this->drawable_object.emplace_back(new Sphere(), ShaderInstances::constant(), std::make_shared<MovementCalculator>(
		std::make_shared<MoveCircle>(
			glm::vec3{ 0.f, 30.f, 0.f },
			40.f),
		glm::vec3{ 30.0, 30.0, 0.0 },
		20.f), drawable_object.size());
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("zombie"), ShaderInstances::phong(), TextureManager::getOrEmplace("zombie", "Textures/zombie.png"), std::make_shared<MovementCalculator>(
		std::make_shared<MoveCircle>(
			glm::vec3{ 0.f, 2.f, 0.f },
			8.f),
		glm::vec3{ 0.0, 3.0, 0.0 },
		20.f), drawable_object.size(), true));

	for (int i = 10; i < draw_objects; i++) {
		if (i % 2 == 0) {
			this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree", "Textures/tree.png"), drawable_object.size(), true));
			float x = ((float)rand() / (float)(RAND_MAX)) * 50;
			float z = ((float)rand() / (float)(RAND_MAX)) * 50;
			this->drawable_object.back().Pos_mov(glm::vec3(x, 0.f, z));
			this->drawable_object.back().Pos_scale(0.3);
		}
		else if (i % 3 == 0)
		{
			this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("zombie"), ShaderInstances::phong(), TextureManager::getOrEmplace("zombie", "Textures/zombie.png"), drawable_object.size(), true));
			float x = ((float)rand() / (float)(RAND_MAX)) * 50;
			float z = ((float)rand() / (float)(RAND_MAX)) * 50;
			float degree = ((float)rand() / (float)(RAND_MAX)) * 50;
			this->drawable_object.back().Pos_mov(glm::vec3(x, 0.f, z));
			this->drawable_object.back().rotate(degree, glm::vec3(0.0f, 1.f, 0.f));
		}
		else {
			this->drawable_object.emplace_back(DrawableObject(new Bushes(), ShaderInstances::phong_no_textures(), drawable_object.size()));
			float x = ((float)rand() / (float)(RAND_MAX)) * 50;
			float z = ((float)rand() / (float)(RAND_MAX)) * 50;
			this->drawable_object.back().Pos_mov(glm::vec3(x, 0.f, z));
			this->drawable_object.back().Pos_scale(3);
		}
	}
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("box"), ShaderInstances::phong_norm(), TextureManager::getOrEmplace("box", "Textures/box_albedo.png", "Textures/box_normal.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(-1, 2.f, 10));

	camera = new Camera();
	camera->registerObserver(ShaderInstances::constant());
	camera->registerObserver(ShaderInstances::phong());
	camera->registerObserver(ShaderInstances::terrain());
	camera->registerObserver(ShaderInstances::phong_no_textures());
	camera->registerObserver(ShaderInstances::phong_norm());
	camera->registerObserver(ShaderInstances::skybox());

	emplaceLight(glm::vec3{ 1.f }, glm::vec3{ -10.f, 50.f, 50.f }, gl::Light::Directional); //SUN OR MOON
	emplaceLight(glm::vec3{ 0.f, 1.f,1.f }, glm::vec3{ -1.f, 2.f, 5.f }, -glm::vec3{ 40.f, 8.f, 0.f }, 9); // FLASHLIGHT (spotlight) - position is set in while loop
	emplaceAmbientLight(glm::vec3{ .1f });

	mouse.instance().registerObserver(*camera);
	mouse.instance().registerObserver(*this);

	Callbacks::Init(window, std::ref(drawable_object), camera, lights[1]); //Initialize Callbacks with drawable object and camera
}

void Scene::Run()
{
	Loop();
}

void Scene::notify(EventType eventType, void* object) {
	if (eventType == EventType::MouseButtonPressed) {
		onButtonPress(((Mouse*)object)->data());
	}
}