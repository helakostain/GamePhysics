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
		applyPhysXTransform();
		//stepPhysics();

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
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere"), ShaderInstances::constant(), TextureManager::getOrEmplace("sphere", "Textures/white_tex.png"), drawable_object.size(), true));
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
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere"), ShaderInstances::constant(), TextureManager::getOrEmplace("sphere", "Textures/white_tex.png"), drawable_object.size(), true));
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
		this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree", "Textures/tree.png"), drawable_object.size(), true));
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
			//createTriangleMeshes(i, j);

			/*if (j == 0)
			{
				createStaticActor(i, j);
			}
			else
			{*/
				createConvexMeshes(i, j);
			//}
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

	/*
	physx::PxRigidDynamic* dynamicObject = gPhysics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f)));
	if (!dynamicObject)
		printf("Failed to create PxRigidDynamic!\n");
	dynamicObject->setMass(10.0f);
	dynamicObject->setRigidBodyFlag(
		physx::PxRigidBodyFlag::eKINEMATIC, true);

	physx::PxShape* shape = physx::PxRigidActorExt::createExclusiveShape(
		*dynamicObject,
		physx::PxTriangleMeshGeometry(triMesh,
			physx::PxMeshScale()),
		*gMaterial
	);

	physx::PxTransform localTm(physx::PxTransform(drawable_object.back().currPosition.x, drawable_object.back().currPosition.y, drawable_object.back().currPosition.z));
	physx::PxRigidDynamic* body = gPhysics->createRigidDynamic(localTm);
	body->attachShape(*shape);
	physx::PxRigidBodyExt::updateMassAndInertia(*body, 10.0f);
	gScene->addActor(*body);

	if (!shape)
		printf("Failed to create PxShape!\n");
	else
		//shape->release();

	triMesh->release();
	//delete[] vertices;
	*/
	//physx::PxTransform tr = physx::PxTransform(*vertices);
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
		meshShape->setLocalPose(physx::PxTransform(physx::PxVec3(drawable_object[j].getModel()->currPosition.x, drawable_object[j].getModel()->currPosition.y, drawable_object[j].getModel()->currPosition.z)));

		//TODO pridat rotace, aktualne je tam jen pozice!
		//glm::quat rotQuat = glm::quat_cast(drawable_object[j].getTransformation()->matrix());
		//meshShape->setLocalPose(physx::PxTransform(physx::PxQuat(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w)));

		physx::PxFilterData filterData;
		filterData.word0 = 1 << 0; // Set the first bit to 1 for kinematic actors
		filterData.word1 = 1 << 1; // Set the second bit to 1 for default collision flag
		meshShape->setSimulationFilterData(filterData);

		meshActor->attachShape(*meshShape);
		meshActor->setMass(16.0f);
		meshActor->setActorFlag(physx::PxActorFlag::eDISABLE_GRAVITY, false);
		gScene->addActor(*meshActor);


		//this moves the actors
		physx::PxTransform currentPose = meshActor->getGlobalPose();
		physx::PxVec3 newVelocity = physx::PxVec3(10.0f, 0.0f, 0.0f); // Set the new velocity to (1, 0, 0)

		// Set the target pose to the current pose with the new velocity
		physx::PxTransform newPose(currentPose.p, currentPose.q);
		newPose.p += newVelocity * (1.0f / 60.0f); // Multiply velocity by timeStep to get the position change
		meshActor->setKinematicTarget(newPose);
	}
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
		meshShape->setLocalPose(physx::PxTransform(physx::PxVec3(drawable_object[j].getModel()->currPosition.x, drawable_object[j].getModel()->currPosition.y, drawable_object[j].getModel()->currPosition.z)));

		//TODO pridat rotace, aktualne je tam jen pozice!
		//glm::quat rotQuat = glm::quat_cast(drawable_object[j].getTransformation()->matrix());
		//meshShape->setLocalPose(physx::PxTransform(physx::PxQuat(rotQuat.x, rotQuat.y, rotQuat.z, rotQuat.w)));

		meshActor->attachShape(*meshShape);
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
		meshShape->setLocalPose(physx::PxTransform(physx::PxVec3(drawable_object[j].getModel()->currPosition.x, drawable_object[j].getModel()->currPosition.y, drawable_object[j].getModel()->currPosition.z)));

		meshActor->attachShape(*meshShape);
		gScene->addActor(*meshActor);
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

void Scene::applyPhysXTransform()
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

			glm::mat4 openMatrix = glm::mat4(matrix.column0.x, matrix.column0.y, matrix.column0.z, matrix.column0.w, matrix.column1.x, matrix.column1.y, matrix.column1.z, matrix.column1.w, matrix.column2.x, matrix.column2.y, matrix.column2.z, matrix.column2.w, matrix.column3.x, matrix.column3.y, -matrix.column3.z, matrix.column3.w);

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
			std::cout << "Before: " << glm::to_string(this->drawable_object[drawObjID].getModel()->currPosition) << std::endl;
			std::cout << "After: " << glm::to_string(glm::vec3(actorPosition.x, actorPosition.y, -actorPosition.z)) << std::endl;

			this->drawable_object[drawObjID].getModel()->applyPhysxTransf(openMatrix, meshID);
			//this->drawable_object[drawObjID].getModel()->applyPhysxTransf(glm::vec3(actorPosition.x, actorPosition.y, -actorPosition.z), meshID);

			//this->drawable_object[1].getModel()->Pos_mov(glm::vec3(actorPosition.x, actorPosition.y, -actorPosition.z));
		}
	}
	//}

	// Release memory for actors array
	delete[] actors;
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
	//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("ground_low"), ShaderInstances::phong(), TextureManager::getOrEmplace("ground_low", "Textures/white_tex.png"), drawable_object.size(), true));
	//this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.0f, 0.f, 10.0f));
	//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("tree"), ShaderInstances::phong(), TextureManager::getOrEmplace("tree", "Textures/tree.png"), drawable_object.size(), true));
	//this->drawable_object.back().Pos_mov(glm::vec3(10.0f, 0.0f, 5.0f));
	
	//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("plane"), ShaderInstances::phong(), TextureManager::getOrEmplace("plane", "Textures/white_tex.png"), drawable_object.size(), true));
	//this->drawable_object.back().Pos_mov(glm::vec3(10.0f, 0.0f, 5.0f));
	//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("car"), ShaderInstances::phong(), TextureManager::getOrEmplace("car", "Textures/white_tex.png"), drawable_object.size(), true));
	//this->drawable_object.back().Pos_mov(glm::vec3(10.0f, 0.0f, 5.0f));
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("wall"), ShaderInstances::phong(), TextureManager::getOrEmplace("wall", "Textures/white_tex.png"), drawable_object.size(), true));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(5.0f, 0.3f, 15.0f));

	//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("brick"), ShaderInstances::phong(), TextureManager::getOrEmplace("brick", "Textures/white_tex.png"), drawable_object.size(), true));
	//this->drawable_object.back().getModel()->Pos_mov(glm::vec3(5.0f, 0.3f, 15.0f));

	//TODO: swiss house nejde prevest do convex meshe
	//this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("swiss_house"), ShaderInstances::phong(), TextureManager::getOrEmplace("swiss_house", "Textures/white_tex.png"), drawable_object.size(), true));
	//this->drawable_object.back().Pos_mov(glm::vec3(25.0f, 0.1f, 5.0f));
	//this->drawable_object.back().rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
	
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
	initPhysics();

	Loop();
	cleanupPhysics();
}

void Scene::notify(EventType eventType, void* object) {
	if (eventType == EventType::MouseButtonPressed) {
		onButtonPress(((Mouse*)object)->data());
	}
}