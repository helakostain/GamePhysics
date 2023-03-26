#include "Scene.hpp"
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

	//Gamma correction
	//glEnable(GL_FRAMEBUFFER_SRGB); 

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

	//applyPhysXStatic();

	const physx::PxVec3 gravity(0, -9.81f, 0); // -9.81 m/s^2 in the negative Y direction

	while (!glfwWindowShouldClose(window)) {  //main while loop for constant rendering of scene
		//physx part
		stepPhysics();

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT); // clear color and depth buffer
		
		skybox->Draw();

		glClear(GL_DEPTH_BUFFER_BIT); //clear window content
		const TimePoint now = std::chrono::high_resolution_clock::now(); //new current time
		const float delta = float(std::chrono::duration_cast<Second>(now - lastTime).count()); //change of before and now time

		ambientLight.apply();
		applyLights();

		//physx part
		applyPhysXTransform(delta, gravity);
		//stepPhysics();

		for (int i = 0; i < this->drawable_object.size(); i++) //apply for all draw objects
		{
			if (i != 1) // HACK:: docasne odpohybovani psa
			{
				this->drawable_object[i].updateObject(delta);
			}
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
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere"), ShaderInstances::constant(), TextureManager::getOrEmplace("sphere", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	//this->drawable_object.back().Pos_mov(pos);
	this->drawable_object.back().getModel()->Pos_mov(pos);
	//this->drawable_object.back().Pos_mov(glm::vec3(0.f, 0.f, (0.1 * (pos.z / abs(pos.z)))));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.f, 0.f, (0.1 * (pos.z / abs(pos.z)))));
	//this->drawable_object.back().Pos_scale(0.25);
	initAndEmplace(light);
	applyLights();
}

void Scene::emplaceLight(glm::vec3 color, glm::vec3 pos, glm::vec3 dir, float cutoff)
{
	std::shared_ptr<ColoredLight> light = std::make_shared<Spotlight>(color, pos, dir, cutoff);
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere"), ShaderInstances::constant(), TextureManager::getOrEmplace("sphere", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	//this->drawable_object.back().Pos_mov(pos);
	this->drawable_object.back().getModel()->Pos_mov(pos);
	//this->drawable_object.back().Pos_mov(glm::vec3(0.f, 0.f, 0.1f));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.f, 0.f, 0.1f));
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
		this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree", "Textures/tree.png"), drawable_object.size(), true, 0));
		//drawable_object.back().Pos_mov(pos);
		drawable_object.back().getModel()->Pos_mov(pos);
		//drawable_object.back().Pos_scale(0.3f);
		drawable_object.back().getModel()->Pos_scale(0.3f);
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

void Scene::initPhysics()
{
	gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

	gPvd = PxCreatePvd(*gFoundation);
	physx::PxPvdTransport* transport = physx::PxDefaultPvdSocketTransportCreate(NULL, 5425, 10);
	gPvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);

	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, physx::PxTolerancesScale(), true, gPvd);
	gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, physx::PxCookingParams(physx::PxTolerancesScale()));

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

	for (int j = 0; j < drawable_object.size(); j++)
	{
		for (int i = 0; i < drawable_object[j].getModel()->meshes.size(); i++)
		{
			if (drawable_object[j].getActorType() == 0)
			{
				createStaticActor(i, j);
			}
			else if (drawable_object[j].getActorType() == 1)
			{
				createConvexMeshes(i, j);
			}
			else if (drawable_object[j].getActorType() == 2)
			{
				createTriangleMeshes(i, j);
			}
		}
	}
	printf("Physx initialized!\n");
	const physx::PxU32 numActors = gScene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
	physx::PxActor** actors = new physx::PxActor * [numActors];
	gScene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC, actors, numActors);
	for (int i = 0; i < numActors; i++)
	{
		int foundID = actorID[actors[i]];
		bool flagFound = false;
		int meshID;
		for (int j = 0; j < drawable_object.size(); j++)
		{
			if (flagFound)
			{
				break;
			}
			for (int k = 0; k < drawable_object[j].getModel()->actorIDs.size(); k++)
			{
				if (std::find(drawable_object[j].getModel()->actorIDs.begin(), drawable_object[j].getModel()->actorIDs.end(), foundID) != drawable_object[j].getModel()->actorIDs.end())
				{
					meshID = foundID;
				}
			}
		}
		//std::cout << "Found actor number: " << i << " with ID " << foundID << " and mesh id: " << meshID << std::endl;
	}
	delete[] actors;
}

void Scene::setupCommonCookingParams(physx::PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData)
{
	// we suppress the triangle mesh remap table computation to gain some speed, as we will not need it 
	// in this snippet
	params.suppressTriangleMeshRemapTable = true;

	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid. 
	// The following conditions are true for a valid triangle mesh :
	//  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
	//  2. There are no large triangles(within specified PxTolerancesScale.)
	// It is recommended to run a separate validation check in debug/checked builds, see below.

	if (!skipMeshCleanup)
		params.meshPreprocessParams &= ~static_cast<physx::PxMeshPreprocessingFlags>(physx::PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
	else
		params.meshPreprocessParams |= physx::PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

	// If DISABLE_ACTIVE_EDGES_PREDOCOMPUTE is set, the cooking does not compute the active (convex) edges, and instead 
	// marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change 
	// the collision behavior, as all edges of the triangle mesh will now be considered active.
	if (!skipEdgeData)
		params.meshPreprocessParams &= ~static_cast<physx::PxMeshPreprocessingFlags>(physx::PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
	else
		params.meshPreprocessParams |= physx::PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

void Scene::createTriangleMeshes(int i, int j)
{
	printf("-----------------------------------------------\n");
	printf("Create triangles mesh kinematic actor using BVH34 midphase: \n\n");
	// Favor cooking speed, skip mesh cleanup, but precompute active edges. Insert into PxPhysics.
	// These settings are suitable for runtime cooking, although selecting more triangles per leaf may reduce
	// runtime performance of simulation and queries. We still need to ensure the triangles 
	// are valid, so we perform a validation check in debug/checked builds.
	physx::PxU32 numVertices = drawable_object[j].getModel()->meshes[i].gVertices.size();
	//const physx::PxVec3* vertices = new physx::PxVec3(drawable_object.front().getModel()->meshes[i].vertices[0].position.x, drawable_object.front().getModel()->meshes[i].vertices[0].position.y, drawable_object.front().getModel()->meshes[i].vertices[0].position.z);
	const physx::PxVec3* vertices = drawable_object[j].getModel()->meshes[i].gVertices.data();
	//const physx::PxVec3* vertices = std::move(drawable_object.front().getModel()->meshes[i].gVertices);
	physx::PxU32 numTriangles = drawable_object[j].getModel()->meshes[i].indices.size() / 3;
	const physx::PxU32* indices = drawable_object[j].getModel()->meshes[i].indices.data();
	//createBV34TriangleMesh(numVertices, vertices, numTriangles, indices, true, false, true, 15);
	physx::PxU64 startTime = physx::shdfnd::Time::getCurrentCounterValue();

	physx::PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(physx::PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(physx::PxU32);

	bool meshDescValid = meshDesc.isValid();
	if (!meshDescValid)
	{
		printf("Mesh description is not valid!\n");
	}

	physx::PxCookingParams params = gCooking->getParams();

	// Create BVH34 midphase
	params.midphaseDesc = physx::PxMeshMidPhase::eBVH34;

	// setup common cooking params
	setupCommonCookingParams(params, true, false);

	// Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
	// and worse cooking performance. Cooking time is better when more triangles per leaf are used.
	params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = 15;

	gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
	// We should check the validity of provided triangles in debug/checked builds though.
	if (true)
	{
		PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
	}
#endif // DEBUG

	physx::PxTriangleMesh* triMesh = NULL;
	physx::PxU32 meshSize = 0;
	bool insertion = false;
	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (insertion)
	{
		triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		physx::PxDefaultMemoryOutputStream outBuffer;
		gCooking->cookTriangleMesh(meshDesc, outBuffer);
		physx::PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
		triMesh = gPhysics->createTriangleMesh(stream);


		meshSize = outBuffer.getSize();
	}

	// Print the elapsed time for comparison
	physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
	float elapsedTime = physx::shdfnd::Time::getCounterFrequency().toTensOfNanos(stopTime - startTime) / (100.0f * 1000.0f);
	printf("\t -----------------------------------------------\n");
	printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
	true ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
	!false ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
	!true ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
	printf("\t\t Num triangles per leaf: %d \n", 15);
	printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
	if (!true)
	{
		printf("\t Mesh size: %d \n", meshSize);
	}

	physx::PxTransform tr = physx::PxTransform(physx::PxVec3(1.0f));
	physx::PxRigidDynamic* meshActor = gPhysics->createRigidDynamic(tr);
	physx::PxShape* meshShape;
	if (meshActor)
	{
		physx::PxTriangleMeshGeometry triGeom;
		triGeom.triangleMesh = triMesh;
		meshShape = gPhysics->createShape(triGeom, *gMaterial, true);
		meshShape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, true);
		meshShape->setFlag(physx::PxShapeFlag::eTRIGGER_SHAPE, false);
		meshActor->setRigidBodyFlag(physx::PxRigidBodyFlag::eKINEMATIC, true);

		physx::PxFilterData filterData;
		filterData.word0 = 1 << 0; // Set the first bit to 1 for kinematic actors
		filterData.word1 = 1 << 1; // Set the second bit to 1 for default collision flag
		meshShape->setSimulationFilterData(filterData);

		meshActor->attachShape(*meshShape);
		meshActor->setMass(16.0f);
		meshActor->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, false);
		physx::PxMat44 matrix = physx::PxMat44(physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].x,
			this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].y,
			this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].x,
				this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].y,
				this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].x,
					this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].y,
					this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].x,
						this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].y,
						this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].z));
		meshActor->setGlobalPose(physx::PxTransform(matrix));
		//gScene->addActor(*meshActor);
		actorID[meshActor] = i;
		drawable_object[j].getModel()->actorIDs.push_back(i);

		/*
		//this moves the actors
		physx::PxTransform currentPose = meshActor->getGlobalPose();
		physx::PxVec3 newVelocity = physx::PxVec3(10.0f, 0.0f, 0.0f); // Set the new velocity to (1, 0, 0)

		// Set the target pose to the current pose with the new velocity
		physx::PxTransform newPose(currentPose.p, currentPose.q);
		newPose.p += newVelocity * (1.0f / 60.0f); // Multiply velocity by timeStep to get the position change
		meshActor->setKinematicTarget(newPose);
		*/

		createCharacter(i, j, meshActor, meshShape); // TODO pro vice meshu nez jeden je potreba presunout ven!
	}
}

void Scene::createCharacter(int i, int j, physx::PxRigidDynamic* actor, physx::PxShape* shape)
{
	// Get the bounds of the triangle mesh actor
	physx::PxBounds3 bounds = actor->getWorldBounds();
	// Compute the height and radius of the capsule controller based on the bounds
	physx::PxVec3 extents = bounds.getExtents();
	float height = extents.y;
	float radius = physx::PxMax(extents.x, extents.z);

	gControllerManager = PxCreateControllerManager(*gScene);
	gControllerManager->setOverlapRecoveryModule(true); //fixes actors on initial possition to not fall through ground etc
	physx::PxCapsuleControllerDesc controllerDesc;
	controllerDesc.height = 3.0f;
	//controllerDesc.height = height;
	controllerDesc.radius = 1.0f;
	//controllerDesc.radius = radius / 2;
	controllerDesc.position = physx::PxExtendedVec3(actor->getGlobalPose().p.x, actor->getGlobalPose().p.y, actor->getGlobalPose().p.z);
	//controllerDesc.upDirection = physx::PxVec3(0.0f, -1.0f, 0.0f); // Set the up direction to simulate gravity acting in the negative Y direction
	controllerDesc.material = gMaterial;
	controllerDesc.slopeLimit = 0.5f;
	controllerDesc.contactOffset = 0.1f;
	controllerDesc.stepOffset = 0.2f;
	//create callback:
	//MyControllerCallback* controllerCallback = new MyControllerCallback();
	//controllerDesc.reportCallback = controllerCallback; maybe to handle hits
	gController = gControllerManager->createController(controllerDesc);
	//controller->setUserData(controllerCallback);
	gController->setUserData(actor);
	gController->setFootPosition(physx::PxExtendedVec3(actor->getGlobalPose().p.x, actor->getGlobalPose().p.y, actor->getGlobalPose().p.z));
	//Callbacks::updateCharacter(std::ref(gController));
}

void Scene::createConvexMeshes(int i, int j)
{
	printf("-----------------------------------------------\n");
	printf("Create convex mesh dynamic actor: \n\n");
	// Favor cooking speed, skip mesh cleanup, but precompute active edges. Insert into PxPhysics.
	// These settings are suitable for runtime cooking, although selecting more triangles per leaf may reduce
	// runtime performance of simulation and queries. We still need to ensure the triangles 
	// are valid, so we perform a validation check in debug/checked builds.
	physx::PxU32 numVertices = drawable_object[j].getModel()->meshes[i].gVertices.size();
	//const physx::PxVec3* vertices = new physx::PxVec3(drawable_object.front().getModel()->meshes[i].vertices[0].position.x, drawable_object.front().getModel()->meshes[i].vertices[0].position.y, drawable_object.front().getModel()->meshes[i].vertices[0].position.z);
	const physx::PxVec3* vertices = drawable_object[j].getModel()->meshes[i].gVertices.data();
	//const physx::PxVec3* vertices = std::move(drawable_object.front().getModel()->meshes[i].gVertices);
	physx::PxU32 numTriangles = drawable_object[j].getModel()->meshes[i].indices.size() / 3;
	const physx::PxU32* indices = drawable_object[j].getModel()->meshes[i].indices.data();
	//createBV34TriangleMesh(numVertices, vertices, numTriangles, indices, true, false, true, 15);
	physx::PxU64 startTime = physx::shdfnd::Time::getCurrentCounterValue();

	physx::PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(physx::PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(physx::PxU32);

	bool meshDescValid = meshDesc.isValid();
	if (!meshDescValid)
	{
		printf("Mesh description is not valid!\n");
	}

	physx::PxCookingParams params = gCooking->getParams();

	// Create BVH34 midphase
	params.midphaseDesc = physx::PxMeshMidPhase::eBVH34;

	// setup common cooking params
	setupCommonCookingParams(params, true, false);

	// Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
	// and worse cooking performance. Cooking time is better when more triangles per leaf are used.
	params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = 15;

	gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
	// We should check the validity of provided triangles in debug/checked builds though.
	if (true)
	{
		PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
	}
#endif // DEBUG


	physx::PxTriangleMesh* triMesh = NULL;
	physx::PxU32 meshSize = 0;
	bool insertion = true;
	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (insertion)
	{
		triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		physx::PxDefaultMemoryOutputStream outBuffer;
		gCooking->cookTriangleMesh(meshDesc, outBuffer);
		physx::PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
		triMesh = gPhysics->createTriangleMesh(stream);


		meshSize = outBuffer.getSize();
	}

	physx::PxConvexMeshDesc convexDesc;
	convexDesc.points.count = triMesh->getNbVertices();
	convexDesc.points.stride = sizeof(physx::PxVec3);
	convexDesc.points.data = triMesh->getVertices();
	convexDesc.flags = physx::PxConvexFlag::eCOMPUTE_CONVEX;
	physx::PxConvexMesh* convexMesh = gCooking->createConvexMesh(convexDesc, gPhysics->getPhysicsInsertionCallback());

	physx::PxTransform tr = physx::PxTransform(physx::PxVec3(1.0f));
	physx::PxRigidDynamic* meshActor = gPhysics->createRigidDynamic(tr);
	physx::PxShape* meshShape;
	if (meshActor)
	{
		physx::PxConvexMeshGeometry convexGeom;
		convexGeom.convexMesh = convexMesh;
		meshShape = gPhysics->createShape(convexGeom, *gMaterial, true);
		if (meshShape == nullptr)
		{
			// Print the elapsed time for comparison
			physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
			float elapsedTime = physx::shdfnd::Time::getCounterFrequency().toTensOfNanos(stopTime - startTime) / (100.0f * 1000.0f);
			printf("\t -----------------------------------------------\n");
			printf("\t Create triangle mesh with %d triangles: FAILED\n", numTriangles);
			printf("\t Elapsed time in ms: %f \n", double(elapsedTime));

			return;
		}
		meshShape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, true);
		meshShape->setFlag(physx::PxShapeFlag::eSCENE_QUERY_SHAPE, true);
		//meshShape->setLocalPose(physx::PxTransform(physx::PxVec3(drawable_object[j].getModel()->currPosition.x, drawable_object[j].getModel()->currPosition.y, drawable_object[j].getModel()->currPosition.z)));

		meshActor->attachShape(*meshShape);
		physx::PxMat44 matrix = physx::PxMat44(physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].x,
			this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].y,
			this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].x,
				this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].y,
				this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].x,
					this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].y,
					this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].x,
						this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].y,
						this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].z));
		meshActor->setGlobalPose(physx::PxTransform(matrix));
		//meshActor->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, false);
		
		physx::PxRigidBodyExt::updateMassAndInertia(*meshActor, 1.6f);
		gScene->addActor(*meshActor);

		actorID[meshActor] = i;
		drawable_object[j].getModel()->actorIDs.push_back(i);

		//TODO: zrychlit!, pridat aby ground byl static a ne dynamic actor, zaridit aby se to tak nerozbijelo, mrknout na github linky v onenotu
	}

	// Print the elapsed time for comparison
	physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
	float elapsedTime = physx::shdfnd::Time::getCounterFrequency().toTensOfNanos(stopTime - startTime) / (100.0f * 1000.0f);
	printf("\t -----------------------------------------------\n");
	printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
	true ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
	!false ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
	!true ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
	printf("\t\t Num triangles per leaf: %d \n", 15);
	printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
	if (!true)
	{
		printf("\t Mesh size: %d \n", meshSize);
	}
}

void Scene::createStaticActor(int i, int j)
{
	printf("-----------------------------------------------\n");
	printf("Create triangles mesh static actor: \n\n");
	// Favor cooking speed, skip mesh cleanup, but precompute active edges. Insert into PxPhysics.
	// These settings are suitable for runtime cooking, although selecting more triangles per leaf may reduce
	// runtime performance of simulation and queries. We still need to ensure the triangles 
	// are valid, so we perform a validation check in debug/checked builds.
	physx::PxU32 numVertices = drawable_object[j].getModel()->meshes[i].gVertices.size();
	//const physx::PxVec3* vertices = new physx::PxVec3(drawable_object.front().getModel()->meshes[i].vertices[0].position.x, drawable_object.front().getModel()->meshes[i].vertices[0].position.y, drawable_object.front().getModel()->meshes[i].vertices[0].position.z);
	const physx::PxVec3* vertices = drawable_object[j].getModel()->meshes[i].gVertices.data();
	//const physx::PxVec3* vertices = std::move(drawable_object.front().getModel()->meshes[i].gVertices);
	physx::PxU32 numTriangles = drawable_object[j].getModel()->meshes[i].indices.size() / 3;
	const physx::PxU32* indices = drawable_object[j].getModel()->meshes[i].indices.data();
	//createBV34TriangleMesh(numVertices, vertices, numTriangles, indices, true, false, true, 15);
	physx::PxU64 startTime = physx::shdfnd::Time::getCurrentCounterValue();

	physx::PxTriangleMeshDesc meshDesc;
	meshDesc.points.count = numVertices;
	meshDesc.points.data = vertices;
	meshDesc.points.stride = sizeof(physx::PxVec3);
	meshDesc.triangles.count = numTriangles;
	meshDesc.triangles.data = indices;
	meshDesc.triangles.stride = 3 * sizeof(physx::PxU32);

	bool meshDescValid = meshDesc.isValid();
	if (!meshDescValid)
	{
		printf("Mesh description is not valid!\n");
	}

	physx::PxCookingParams params = gCooking->getParams();

	// Create BVH34 midphase
	params.midphaseDesc = physx::PxMeshMidPhase::eBVH34;

	// setup common cooking params
	setupCommonCookingParams(params, true, false);

	// Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
	// and worse cooking performance. Cooking time is better when more triangles per leaf are used.
	params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = 15;

	gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
	// If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. 
	// We should check the validity of provided triangles in debug/checked builds though.
	if (true)
	{
		PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
	}
#endif // DEBUG


	physx::PxTriangleMesh* triMesh = NULL;
	physx::PxU32 meshSize = 0;
	bool insertion = true;
	// The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
	if (insertion)
	{
		triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
	}
	else
	{
		physx::PxDefaultMemoryOutputStream outBuffer;
		gCooking->cookTriangleMesh(meshDesc, outBuffer);
		physx::PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
		triMesh = gPhysics->createTriangleMesh(stream);


		meshSize = outBuffer.getSize();
	}

	physx::PxTransform tr = physx::PxTransform(physx::PxVec3(1.0f));
	physx::PxRigidStatic* meshActor = gPhysics->createRigidStatic(tr);
	physx::PxShape* meshShape;
	if (meshActor)
	{
		physx::PxTriangleMeshGeometry triGeom;
		triGeom.triangleMesh = triMesh;
		meshShape = gPhysics->createShape(triGeom, *gMaterial, true);
		meshShape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, true);
		//meshShape->setLocalPose(physx::PxTransform(physx::PxVec3(drawable_object[j].getModel()->currPosition.x, drawable_object[j].getModel()->currPosition.y, drawable_object[j].getModel()->currPosition.z)));

		meshActor->attachShape(*meshShape);
		physx::PxMat44 matrix = physx::PxMat44(physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].x,
			this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].y,
			this->drawable_object[j].getModel()->getTransformation(i)->matrix()[0].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].x,
				this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].y,
				this->drawable_object[j].getModel()->getTransformation(i)->matrix()[1].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].x,
					this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].y,
					this->drawable_object[j].getModel()->getTransformation(i)->matrix()[2].z), physx::PxVec3(this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].x,
						this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].y,
						this->drawable_object[j].getModel()->getTransformation(i)->matrix()[3].z));
		meshActor->setGlobalPose(physx::PxTransform(matrix));
		gScene->addActor(*meshActor);
		//TODO prirazeni rozbije scenu
		//actorID[meshActor] = i;
		//drawable_object[j].getModel()->actorIDs.push_back(i);
	}

	// Print the elapsed time for comparison
	physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
	float elapsedTime = physx::shdfnd::Time::getCounterFrequency().toTensOfNanos(stopTime - startTime) / (100.0f * 1000.0f);
	printf("\t -----------------------------------------------\n");
	printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
	true ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
	!false ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
	!true ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
	printf("\t\t Num triangles per leaf: %d \n", 15);
	printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
	if (!true)
	{
		printf("\t Mesh size: %d \n", meshSize);
	}
}

void Scene::stepPhysics()
{
	gScene->simulate(1.0f / 60.0f);
	gScene->fetchResults(true);
}

void Scene::cleanupPhysics()
{
	if (gControllerManager)
	{
		gControllerManager->purgeControllers();
		gControllerManager->release();
	}
	gScene->release();
	gDispatcher->release();
	gPhysics->release();
	gCooking->release();
	if (gPvd)
	{
		physx::PxPvdTransport* transport = gPvd->getTransport();
		gPvd->release();	gPvd = NULL;
		transport->release();
	}
	gFoundation->release();

	printf("PhysX done.\n");
}

void Scene::applyPhysXTransform(const float delta, const physx::PxVec3 gravity)
{
	const physx::PxU32 numActors = gScene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
	physx::PxActor** actors = new physx::PxActor * [numActors];
	gScene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC, actors, numActors);

	// Iterate through actors and get their position
	for (physx::PxU32 i = 0; i < numActors; i++)
	{
	//if (actors[1]->is<physx::PxRigidActor>()) // Check if actor is a rigid actor
	//{
		if (auto found = actorID.find(actors[i]); found != actorID.end())
		{
			int foundID = actorID[actors[i]];
			//std::cout << "Actor num: " << foundID << " was found." << std::endl;
			physx::PxRigidActor* rigidActor = static_cast<physx::PxRigidActor*>(actors[i]);
			physx::PxTransform actorTransform = rigidActor->getGlobalPose();
			physx::PxMat44 matrix = physx::PxMat44(actorTransform);
			physx::PxVec3 actorPosition = actorTransform.p;

			glm::mat4 openMatrix = glm::mat4(matrix.column0.x, matrix.column0.y, matrix.column0.z, matrix.column0.w, matrix.column1.x, matrix.column1.y, matrix.column1.z, matrix.column1.w, matrix.column2.x, matrix.column2.y, matrix.column2.z, matrix.column2.w, matrix.column3.x, matrix.column3.y, matrix.column3.z, matrix.column3.w);

			bool flagFound = false;
			int meshID;
			int drawObjID = 0;
			for (int j = 0; j < drawable_object.size(); j++)
			{
				if (flagFound)
				{
					break;
				}
				for (int k = 0; k < drawable_object[j].getModel()->actorIDs.size(); k++)
				{
					if (std::find(drawable_object[j].getModel()->actorIDs.begin(), drawable_object[j].getModel()->actorIDs.end(), foundID) != drawable_object[j].getModel()->actorIDs.end())
					{
						meshID = foundID;
						drawObjID = j;
					}
				}
			}
			//std::cout << "Found actor number: " << i << " with ID " << foundID << " and mesh id: " << meshID << std::endl;
			//std::cout << "Before: " << glm::to_string(this->drawable_object[drawObjID].getModel()->getTransformation(meshID)->matrix()) << std::endl;
			//std::cout << "Before: " << glm::to_string(this->drawable_object[drawObjID].getModel()->currPosition) << std::endl;
			//std::cout << "After: " << glm::to_string(glm::vec3(matrix.column3.x, matrix.column3.y, -matrix.column3.z)) << std::endl;	
			this->drawable_object[drawObjID].getModel()->applyPhysxTransf(openMatrix, meshID);
			
			//this->drawable_object[drawObjID].getModel()->applyPhysxTransf(glm::vec3(actorPosition.x, actorPosition.y, -actorPosition.z), meshID);

			//this->drawable_object[1].getModel()->Pos_mov(glm::vec3(actorPosition.x, actorPosition.y, -actorPosition.z));
		}
	}
	//move character
	physx::PxRigidActor* character = gController->getActor();
	physx::PxTransform actorTransform = character->getGlobalPose();
	physx::PxMat44 matrix = physx::PxMat44(actorTransform);
	physx::PxVec3 actorPosition = actorTransform.p;

	glm::mat4 openMatrix = glm::mat4(matrix.column0.x, matrix.column0.y, matrix.column0.z, matrix.column0.w, matrix.column1.x, matrix.column1.y, matrix.column1.z, matrix.column1.w, matrix.column2.x, matrix.column2.y, matrix.column2.z, matrix.column2.w, matrix.column3.x, matrix.column3.y, matrix.column3.z, matrix.column3.w);
	for (int i = 0; i < this->drawable_object[1].getModel()->meshes.size(); i++) // HACK: natvrdo pozice modelu
	{
		this->drawable_object[1].getModel()->applyPhysxTransf(openMatrix, i);
		this->drawable_object[1].getModel()->rotate(270.0f, glm::vec3(0.0f, 0.0f, 1.0f));
		//this->drawable_object[1].getModel()->Pos_mov(glm::vec3(0.0f, -1.6f, 0.0f));
	}
	//camera->setCamera(openMatrix);
	openMatrix[3].y = openMatrix[3].y + 1.0f;
	camera->setPosition(openMatrix[3]); // this moves camera to position of character controller
	camera->update(delta);

	// Call the move method on the controller
	physx::PxControllerCollisionFlags collisionFlags;
	physx::PxVec3 movement(0.0f, 0.0f, 0.0f);
	physx::PxControllerState controllerState;
	gController->getState(controllerState);
	controllerState.deltaXP.y += gravity.y * delta;
	physx::PxVec3 forwardDir(0.0f, 0.0f, 0.0f);

	// Add the resulting vector to the character controller's position to move it in the desired direction
	glm::vec3 cameraForward = camera->direction();
	glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
	switch (this->drawable_object[1].getModel()->moved) // HACK: natvrdo pozice modelu
	{
	case 0:
		collisionFlags = gController->move(controllerState.deltaXP, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 1:
		//movement = physx::PxVec3(0.0f, 0.0f, 1.0f) + controllerState.deltaXP; // move one unit forward
		forwardDir = physx::PxVec3(camera->view()[2].x, camera->view()[2].y, -camera->view()[2].z); // Extract the forward direction vector from the view matrix
		//forwardDir.normalize(); // Normalize the vector to ensure it has unit length
		//forwardDir *= delta; // Scale the vector by your desired speed
		movement = forwardDir + controllerState.deltaXP;
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 2:
		//movement = physx::PxVec3(0.0f, 0.0f, -1.0f) + controllerState.deltaXP; // move one unit backward
		forwardDir = physx::PxVec3(-camera->view()[2].x, camera->view()[2].y, camera->view()[2].z);
		movement = forwardDir + controllerState.deltaXP;
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 3:
		//movement = physx::PxVec3(1.0f, 0.0f, 0.0f) + controllerState.deltaXP; // move one unit left
		glm::vec3 cameraLeft = glm::normalize(glm::cross(cameraForward, cameraUp));
		movement = physx::PxVec3(-cameraLeft.x, cameraLeft.y, -cameraLeft.z) + controllerState.deltaXP; // move one unit sideways to the left
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 4:
		//movement = physx::PxVec3(-1.0f, 0.0f, 0.0f) + controllerState.deltaXP; // move one unit right
		glm::vec3 cameraRight = glm::normalize(glm::cross(cameraForward, cameraUp));
		movement = physx::PxVec3(cameraRight.x, cameraRight.y, cameraRight.z) + controllerState.deltaXP; // move one unit sideways to the left
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	default:
		collisionFlags = gController->move(controllerState.deltaXP, 0.01f, delta, physx::PxControllerFilters());
		break;
	}
	//drawable_object[1].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
	if (this->drawable_object[1].getModel()->moved != this->drawable_object[1].getModel()->last_moved && this->drawable_object[1].getModel()->moved != 0) // HACK: natvrdo pozice modelu
	{
		//TODO tohle nefunguje, kdovi proc
		switch (this->drawable_object[1].getModel()->moved)
		{
		case 1:
			switch (this->drawable_object[1].getModel()->last_moved)
			{
			case 2:
				this->drawable_object[1].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 3:
				this->drawable_object[1].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 4:
				this->drawable_object[1].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		case 2:
			switch (this->drawable_object[1].getModel()->last_moved)
			{
			case 1:
				this->drawable_object[1].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 3:
				this->drawable_object[1].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 4:
				this->drawable_object[1].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		case 3:
			switch (this->drawable_object[1].getModel()->last_moved)
			{
			case 1:
				this->drawable_object[1].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 2:
				this->drawable_object[1].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 4:
				this->drawable_object[1].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		case 4:
			switch (this->drawable_object[1].getModel()->last_moved)
			{
			case 1:
				this->drawable_object[1].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 2:
				this->drawable_object[1].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 3:
				this->drawable_object[1].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
		this->drawable_object[1].getModel()->last_moved = drawable_object[1].getModel()->moved;
	}

	this->drawable_object[1].getModel()->moved = 0; // HACK: natvrdo pozice modelu

	if (this->drawable_object[1].getModel()->shot) // HACK: natvrdo pozice modelu
	{
		//shootBall(gController->getActor()->getGlobalPose(), physx::PxSphereGeometry(1.0f), gController->getActor()->getGlobalPose().rotate(physx::PxVec3(camera->direction().x, camera->direction().y, camera->direction().z))*10);
		/*glm::vec3 cameraDir = camera->direction();
		glm::mat4 cameraRotMat = glm::lookAt(glm::vec3(0.0f), -cameraDir, glm::vec3(0.0f, 1.0f, 0.0f));
		physx::PxVec3 physxDir = physx::PxVec3(cameraRotMat[2][0], cameraRotMat[2][1], cameraRotMat[2][2]);
		physx::PxVec3 velocity = physxDir * 10.0f;
		physx::PxMat44 matri = physx::PxMat44(
			physx::PxVec4(camera->view()[0].x, camera->view()[0].y, camera->view()[0].z, camera->view()[0].w),
			physx::PxVec4(camera->view()[1].x, camera->view()[1].y, camera->view()[1].z, camera->view()[1].w),
			physx::PxVec4(camera->view()[2].x, camera->view()[2].y, camera->view()[3].z, camera->view()[2].w),
			physx::PxVec4(camera->view()[3].x, camera->view()[3].y, camera->view()[3].z, camera->view()[3].w)
		);
		physx::PxTransform camTram = physx::PxTransform(matri);
		
		physx::PxMat44 matrix = physx::PxMat44(
			physx::PxVec3(camera->view()[0].x, camera->view()[0].y, camera->view()[0].z),
			physx::PxVec3(camera->view()[1].x, camera->view()[1].y, camera->view()[1].z),
			physx::PxVec3(camera->view()[2].x, camera->view()[2].y, camera->view()[2].z),
			physx::PxVec3(camera->view()[3].x, camera->view()[3].y, camera->view()[3].z));
		physx::PxTransform trans = physx::PxTransform(matrix);
		trans.p.y += 3.0f;
		shootBall(trans, physx::PxSphereGeometry(1.0f), matrix.rotate(physx::PxVec3(0, 0, -1)) * 10);
		*/
		physx::PxVec3 pxCameraPos(camera->position().x, camera->position().y, camera->position().z);
		physx::PxVec3 pxCameraDir(camera->direction().x, camera->direction().y, camera->direction().z);
		float sphereRadius = 1.0f; // Set the radius of the sphere.
		physx::PxVec3 spherePos = pxCameraPos + (pxCameraDir * (sphereRadius + 1.0f)); // Add 1 unit to avoid collision with the camera.
		physx::PxTransform sphereTransform(spherePos);
		float sphereSpeed = 10.0f; // Set the speed of the sphere.
		physx::PxVec3 sphereVelocity = pxCameraDir * sphereSpeed;
		physx::PxSphereGeometry sphereGeometry(sphereRadius);
		physx::PxRigidDynamic* new_ball = shootBall(sphereTransform, sphereGeometry, sphereVelocity);
		//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere"), ShaderInstances::constant(), TextureManager::getOrEmplace("sphere", "Textures/white_tex.png"), drawable_object.size(), true, 0));
		//TODO: crash na meshID pri vlozeni ball do draw_obj
		//actorID[new_ball] = num_balls;
		//num_balls++;
		//drawable_object.back().getModel()->actorIDs.push_back(num_balls);
		this->drawable_object[1].getModel()->shot = false;  // HACK: natvrdo pozice modelu
	}
	//}

	// Release memory for actors array
	delete[] actors;
}

void Scene::applyPhysXStatic()
{
	const physx::PxU32 numActors = gScene->getNbActors(physx::PxActorTypeFlag::eRIGID_STATIC);
	physx::PxActor** actors = new physx::PxActor * [numActors];
	gScene->getActors(physx::PxActorTypeFlag::eRIGID_STATIC, actors, numActors);

	// Iterate through actors and get their position
	for (physx::PxU32 i = 0; i < numActors; i++)
	{
		//if (actors[1]->is<physx::PxRigidActor>()) // Check if actor is a rigid actor
		//{
		if (auto found = actorID.find(actors[i]); found != actorID.end())
		{
			int foundID = actorID[actors[i]];
			//std::cout << "Actor num: " << foundID << " was found." << std::endl;
			physx::PxRigidActor* rigidActor = static_cast<physx::PxRigidActor*>(actors[i]);
			physx::PxTransform actorTransform = rigidActor->getGlobalPose();
			physx::PxMat44 matrix = physx::PxMat44(actorTransform);
			physx::PxVec3 actorPosition = actorTransform.p;

			glm::mat4 openMatrix = glm::mat4(matrix.column0.x, matrix.column0.y, matrix.column0.z, matrix.column0.w, matrix.column1.x, matrix.column1.y, matrix.column1.z, matrix.column1.w, matrix.column2.x, matrix.column2.y, matrix.column2.z, matrix.column2.w, matrix.column3.x, matrix.column3.y, matrix.column3.z, matrix.column3.w);

			bool flagFound = false;
			int meshID = 0;
			int drawObjID = 0;
			for (int j = 0; j < drawable_object.size(); j++)
			{
				if (flagFound)
				{
					break;
				}
				for (int k = 0; k < drawable_object[j].getModel()->actorIDs.size(); k++)
				{
					if (std::find(drawable_object[j].getModel()->actorIDs.begin(), drawable_object[j].getModel()->actorIDs.end(), foundID) != drawable_object[j].getModel()->actorIDs.end())
					{
						meshID = foundID;
						drawObjID = j;
					}
				}
			}
			//std::cout << "Found actor number: " << i << " with ID " << foundID << " and mesh id: " << meshID << std::endl;
			//std::cout << "Before: " << glm::to_string(this->drawable_object[drawObjID].getModel()->getTransformation(meshID)->matrix()) << std::endl;
			//std::cout << "Before: " << glm::to_string(this->drawable_object[drawObjID].getModel()->currPosition) << std::endl;
			//std::cout << "After: " << glm::to_string(glm::vec3(matrix.column3.x, matrix.column3.y, -matrix.column3.z)) << std::endl;	
			this->drawable_object[drawObjID].getModel()->applyPhysxTransf(openMatrix, meshID);

			//this->drawable_object[drawObjID].getModel()->applyPhysxTransf(glm::vec3(actorPosition.x, actorPosition.y, -actorPosition.z), meshID);

			//this->drawable_object[1].getModel()->Pos_mov(glm::vec3(actorPosition.x, actorPosition.y, -actorPosition.z));
		}
	}
	// Release memory for actors array
	delete[] actors;
}

physx::PxRigidDynamic* Scene::shootBall(const physx::PxTransform& t, const physx::PxGeometry& geometry, const physx::PxVec3& velocity)
{
	/*
	physx::PxRigidDynamic* dynamic = physx::PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
	*/
	physx::PxRigidDynamic* dynamic = gPhysics->createRigidDynamic(t);
	physx::PxShape* meshShape;
	meshShape = gPhysics->createShape(geometry, *gMaterial, true);
	meshShape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, true);
	meshShape->setFlag(physx::PxShapeFlag::eSCENE_QUERY_SHAPE, true);
	dynamic->attachShape(*meshShape);
	physx::PxRigidBodyExt::updateMassAndInertia(*dynamic, 1.6f);
	dynamic->setAngularDamping(0.5f);
	dynamic->setLinearVelocity(velocity);
	gScene->addActor(*dynamic);
	return dynamic;
}

void Scene::createForest()
{
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree0"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree0", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(-20.0f, 0.3f, -10.0f));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree1"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree0", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(-24.0f, 0.3f, -13.0f));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree2"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree0", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(-18.0f, 0.3f, -17.0f));
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
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("ground_with_fence"), ShaderInstances::phong(), TextureManager::getOrEmplace("ground_with_fence", "Textures/white_tex.png"), drawable_object.size(), true, 0));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("dog"), ShaderInstances::phong(), TextureManager::getOrEmplace("dog", "Textures/white_tex.png"), drawable_object.size(), true, 2));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.0f, 0.3f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("wall2"), ShaderInstances::phong(), TextureManager::getOrEmplace("wall2", "Textures/white_tex.png"), drawable_object.size(), true, 1));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(5.0f, 0.3f, 15.0f));
	
	createForest();

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("garage"), ShaderInstances::phong(), TextureManager::getOrEmplace("garage", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(-20.0f, 0.3f, 1.0f));
	//this->drawable_object.back().getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("laboratory"), ShaderInstances::phong(), TextureManager::getOrEmplace("laboratory", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(-20.0f, 0.3f, 22.0f));
	this->drawable_object.back().getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("pizza"), ShaderInstances::phong(), TextureManager::getOrEmplace("pizza", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(20.0f, 0.4f, 22.0f));
	this->drawable_object.back().getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));

	//TODO: swiss house nejde prevest do convex meshe
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("swiss_house"), ShaderInstances::phong(), TextureManager::getOrEmplace("swiss_house", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(25.0f, 0.3f, 0.0f));
	//this->drawable_object.back().getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
	
	camera = new Camera();
	camera->registerObserver(ShaderInstances::constant());
	camera->registerObserver(ShaderInstances::phong());
	camera->registerObserver(ShaderInstances::terrain());
	camera->registerObserver(ShaderInstances::phong_no_textures());
	camera->registerObserver(ShaderInstances::phong_norm());
	camera->registerObserver(ShaderInstances::skybox());

	emplaceLight(glm::vec3{ 1.f }, glm::vec3{ -10.f, 50.f, -20.f }, gl::Light::Directional); //SUN OR MOON
	emplaceLight(glm::vec3{ 1.f }, glm::vec3{ 10.f, 50.f, -20.f }, gl::Light::Directional); //SUN OR MOON
	emplaceLight(glm::vec3{ 0.f, 1.f,1.f }, glm::vec3{ -1.f, 2.f, 5.f }, -glm::vec3{ 40.f, 8.f, 0.f }, 0); // FLASHLIGHT (spotlight) - position is set in while loop, zero at end is turned off
	emplaceAmbientLight(glm::vec3{ .1f });

	mouse.instance().registerObserver(*camera);
	mouse.instance().registerObserver(*this);

	Callbacks::Init(window, std::ref(drawable_object), camera, lights[1]); //Initialize Callbacks with drawable object and camera
}

void Scene::Run()
{
	initPhysics();

	Loop();
	cleanupPhysics();
}

void Scene::notify(EventType eventType, void* object) {
	if (eventType == EventType::MouseButtonPressed) {
		onButtonPress(((Mouse*)object)->data());
	}
}