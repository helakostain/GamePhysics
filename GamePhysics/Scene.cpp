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

#define draw_objects 10

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
		//physx part
		stepPhysics(true);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT); // clear color and depth buffer
		
		skybox->Draw();

		glClear(GL_DEPTH_BUFFER_BIT); //clear window content
		const TimePoint now = std::chrono::high_resolution_clock::now(); //new current time
		const float delta = std::chrono::duration_cast<Second>(now - lastTime).count(); //change of before and now time

		ambientLight.apply();
		applyLights();

		//physx part
		
		physx::PxScene* scene;
		PxGetPhysics().getScenes(&scene, 1);
		physx::PxU32 nbActors = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC);
		if (nbActors)
		{
			std::vector<physx::PxRigidActor*> actors(nbActors);
			scene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC | physx::PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<physx::PxActor**>(&actors[0]), nbActors);
			applyPhysXTransform();
		}

		for (int i = 0; i < this->drawable_object.size(); i++) //apply for all draw objects
		{
			this->drawable_object[i].updateObject(delta);
			lastTime = now;
		}
		//initPhysics(true);

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

physx::PxRigidDynamic* Scene::createDynamic(const physx::PxTransform& t, const physx::PxGeometry& geometry, const physx::PxVec3& velocity)
{
	physx::PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

void Scene::createStack(const physx::PxTransform& t, physx::PxU32 size, physx::PxReal halfExtent)
{
	physx::PxShape* shape = gPhysics->createShape(physx::PxBoxGeometry(halfExtent, halfExtent, halfExtent), *gMaterial);
	for (physx::PxU32 i = 0; i < size; i++)
	{
		for (physx::PxU32 j = 0; j < size - i; j++)
		{
			physx::PxTransform localTm(physx::PxVec3(physx::PxReal(j * 2) - physx::PxReal(size - i), physx::PxReal(i * 2 + 1), 0) * halfExtent);
			physx::PxRigidDynamic* body = gPhysics->createRigidDynamic(t.transform(localTm));
			body->attachShape(*shape);
			physx::PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
			gScene->addActor(*body);
		}
	}
	shape->release();
}

void Scene::initPhysics(bool interactive)
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate(NULL, 5425, 10);
	gPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, physx::PxTolerancesScale(), true, gPvd);

	physx::PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
	gScene = gPhysics->createScene(sceneDesc);

	physx::PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
	if (pvdClient)
	{
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
		pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
	}
	gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

	physx::PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, physx::PxPlane(0, 1, 0, 0), *gMaterial);
	gScene->addActor(*groundPlane);

	//for (physx::PxU32 i = 0; i < 5; i++)
	//	createStack(physx::PxTransform(physx::PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);

	for (int i = 0; i < drawable_object.front().getModel()->meshes.size(); i++)
	{
		if (!toPhysxActor(i))
		{
			printf("Failed to pass all models to actors!");
		}
	}

	//if (!interactive)
	//	createDynamic(physx::PxTransform(physx::PxVec3(0, 40, 100)), physx::PxSphereGeometry(10), physx::PxVec3(0, -50, -100));
}

void Scene::stepPhysics(bool)
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void Scene::cleanupPhysics(bool)
{
	gScene->release();
	gDispatcher->release();
	gPhysics->release();
	if (gPvd)
	{
		physx::PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		transport->release();
	}
	gFoundation->release();

	printf("SnippetHelloWorld done.\n");
}

void Scene::keyPress(unsigned char key, const physx::PxTransform& camera)
{
	switch (toupper(key))
	{
	case 'B':	createStack(physx::PxTransform(physx::PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f); break;
	case ' ':	createDynamic(camera, physx::PxSphereGeometry(3.0f), camera.rotate(physx::PxVec3(0, 0, -1)) * 200);	break;
	}
}

bool Scene::toPhysxActor(int i)
{
	physx::PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = drawable_object.front().getModel()->meshes[i].vertices.size();
	meshDesc.points.stride = sizeof(Vertex);
	meshDesc.points.data = &drawable_object.front().getModel()->meshes[i].vertices[0];
	meshDesc.triangles.count = drawable_object.front().getModel()->meshes[i].indices.size();
	meshDesc.triangles.stride = 3 * sizeof(uint32_t);
	meshDesc.triangles.data = &drawable_object.front().getModel()->meshes[i].indices[0];

	physx::PxDefaultMemoryOutputStream writeBuffer;
	physx::PxTriangleMeshCookingResult::Enum result;
	physx::PxCookingParams cookingParams = physx::PxCookingParams(physx::PxTolerancesScale());
	cookingParams.meshWeldTolerance = 0.1f;
	cookingParams.meshPreprocessParams = physx::PxMeshPreprocessingFlag::eWELD_VERTICES;
	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, cookingParams);
	gScene->setVisualizationParameter(physx::PxVisualizationParameter::eSCALE, 2);
	if (!mCooking)
		printf("PxCreateCooking failed!");
	if (mCooking->validateTriangleMesh(meshDesc))
	{
		bool status = mCooking->cookTriangleMesh(meshDesc, writeBuffer, &result);
		if (!status)
			printf("cooking failed!\n");
			return false;
	}
	else
	{
		printf("Validation of cooking failed!\n");
		return false;
	}
	

	physx::PxDefaultMemoryInputData readBuffer(writeBuffer.getData(),
		writeBuffer.getSize());
	physx::PxTriangleMesh* triangleMesh =
		gPhysics->createTriangleMesh(readBuffer);

	physx::PxRigidDynamic* dynamicObject = gPhysics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f)));
	if (!dynamicObject)
		return false;
	dynamicObject->setMass(10.0f);
	dynamicObject->setRigidBodyFlag(
		physx::PxRigidBodyFlag::eKINEMATIC, true);

	physx::PxShape* shape = physx::PxRigidActorExt::createExclusiveShape(
		*dynamicObject,
		physx::PxTriangleMeshGeometry(triangleMesh,
			physx::PxMeshScale()),
		*gMaterial
	);

	physx::PxTransform localTm( physx::PxTransform(drawable_object.back().currPosition.x, drawable_object.back().currPosition.y, drawable_object.back().currPosition.z));
	physx::PxRigidDynamic* body = gPhysics->createRigidDynamic(localTm);
	body->attachShape(*shape);
	physx::PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
	gScene->addActor(*body);

	if (!shape)
		return false;
	else
		shape->release();
		return true;
}

void Scene::applyPhysXTransform()
{
}


Scene::Scene(GLFWwindow* in_window)
{
	this->window = in_window;

	static std::vector<std::string> cubemapTextures{
		"Textures/cubemap/posx.jpg",
		"Textures/cubemap/negx.jpg",
		"Textures/cubemap/posy.jpg",
		"Textures/cubemap/negy.jpg",
		"Textures/cubemap/posz.jpg",
		"Textures/cubemap/negz.jpg"
	};

	srand(time(NULL));
	this->skybox = std::make_shared<Skybox>(TextureManager::cubeMap("skybox", cubemapTextures));
	//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("ground"), ShaderInstances::phong(), TextureManager::getOrEmplace("ground", "Textures/white_tex.png"), drawable_object.size(), true));
	//this->drawable_object.back().Pos_mov(glm::vec3(0, 0.f, 10));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree", "Textures/tree.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(10, 0.0f, 5));
	/*
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("car"), ShaderInstances::phong(), TextureManager::getOrEmplace("car", "Textures/white_tex.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(10, 0.0f, 5));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("car"), ShaderInstances::phong(), TextureManager::getOrEmplace("car", "Textures/white_tex.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(10, 0.0f, 5));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("wall"), ShaderInstances::phong(), TextureManager::getOrEmplace("wall", "Textures/white_tex.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(5, 0.0f, 15));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("swiss_house"), ShaderInstances::phong(), TextureManager::getOrEmplace("swiss_house", "Textures/white_tex.png"), drawable_object.size(), true));
	this->drawable_object.back().Pos_mov(glm::vec3(25, 0.1f, 5));
	this->drawable_object.back().rotate(90.0f, glm::vec3(0, 1, 0));
	*/
	
	camera = new Camera();
	camera->registerObserver(ShaderInstances::constant());
	camera->registerObserver(ShaderInstances::phong());
	camera->registerObserver(ShaderInstances::terrain());
	camera->registerObserver(ShaderInstances::phong_no_textures());
	camera->registerObserver(ShaderInstances::phong_norm());
	camera->registerObserver(ShaderInstances::skybox());

	emplaceLight(glm::vec3{ 1.f }, glm::vec3{ 10.f, 50.f, -50.f }, gl::Light::Directional); //SUN OR MOON
	emplaceLight(glm::vec3{ 0.f, 1.f,1.f }, glm::vec3{ -1.f, 2.f, 5.f }, -glm::vec3{ 40.f, 8.f, 0.f }, 0); // FLASHLIGHT (spotlight) - position is set in while loop, zero at end is turned off
	emplaceAmbientLight(glm::vec3{ .1f });

	mouse.instance().registerObserver(*camera);
	mouse.instance().registerObserver(*this);

	Callbacks::Init(window, std::ref(drawable_object), camera, lights[1]); //Initialize Callbacks with drawable object and camera
}

void Scene::Run()
{
	static const physx::PxU32 frameCount = 100;
	initPhysics(false);
	//for (physx::PxU32 i = 0; i < frameCount; i++)
	//	stepPhysics(false);
	//cleanupPhysics(false);
	
	Loop();
	cleanupPhysics(true);
}

void Scene::notify(EventType eventType, void* object) {
	if (eventType == EventType::MouseButtonPressed) {
		onButtonPress(((Mouse*)object)->data());
	}
}