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
	glDisable(GL_FRAMEBUFFER_SRGB);

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

		for (int i = 0; i < this->drawable_object.size(); i++) //apply for all draw objects
		{
			if (!this->drawable_object[i].getModel()->isCharacter) // HACK:: docasne odpohybovani psa
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
	initAndEmplace(light);
	applyLights();
}

void Scene::emplaceLight(glm::vec3 color, glm::vec3 pos, glm::vec3 dir, float cutoff)
{
	std::shared_ptr<ColoredLight> light = std::make_shared<Spotlight>(color, pos, dir, cutoff);
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
		drawable_object.back().getModel()->Pos_mov(pos);
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

	physx::PxCudaContextManagerDesc cudaContextManagerDesc;
	if (cudaON)
	{
		cudaContextManagerDesc.interopMode = physx::PxCudaInteropMode::OGL_INTEROP;
		gCudaContextManager = PxCreateCudaContextManager(*gFoundation, cudaContextManagerDesc, PxGetProfilerCallback());	//Create the CUDA context manager, required for GRB to dispatch CUDA kernels.
		if (gCudaContextManager)
		{
			if (!gCudaContextManager->contextIsValid())
			{
				gCudaContextManager->release();
				gCudaContextManager = NULL;
				cudaON = false;
			}
		}
	}

	physx::PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
	gDispatcher = physx::PxDefaultCpuDispatcherCreate(6); //Create a CPU dispatcher using 6 worker threads
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = physx::PxDefaultSimulationFilterShader;
	sceneDesc.broadPhaseType = physx::PxBroadPhaseType::eABP;

	if (cudaON)
	{
		sceneDesc.cudaContextManager = gCudaContextManager;		//Set the CUDA context manager, used by GRB.

		sceneDesc.flags |= physx::PxSceneFlag::eENABLE_GPU_DYNAMICS;	//Enable GPU dynamics - without this enabled, simulation (contact gen and solver) will run on the CPU.
		sceneDesc.flags |= physx::PxSceneFlag::eENABLE_PCM;			//Enable PCM. PCM NP is supported on GPU. Legacy contact gen will fall back to CPU
		sceneDesc.flags |= physx::PxSceneFlag::eENABLE_STABILIZATION;	//Improve solver stability by enabling post-stabilization.
		sceneDesc.broadPhaseType = physx::PxBroadPhaseType::eGPU;		//Enable GPU broad phase. Without this set, broad phase will run on the CPU.
		sceneDesc.gpuMaxNumPartitions = 8;						//Defines the maximum number of partitions used by the solver. Only power-of-2 values are valid. 
		//A value of 8 generally gives best balance between performance and stability.
	}

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

	//TODO: fixnout ty ozubena kola
	//int* gears = new int(0);
	//int* sticks = new int(0);
	//bool jointActorAssign = false;
	for (int j = 0; j < drawable_object.size(); j++)
	{
		for (int i = 0; i < drawable_object[j].getModel()->meshes.size(); i++)
		{
			//TODO: fixnout ty ozubena kola
			/*
			if ((j == drawable_object.size() - 3) || (j == drawable_object.size() - 2))
			{
				createWheelActor(i, j, gears, sticks);
			}
			else */if (drawable_object[j].getActorType() == 0)
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
	//TODO: fixnout ty ozubena kola
	//physx::PxRevoluteJoint* joint = physx::PxRevoluteJointCreate(*gPhysics, stick1, stick1->getGlobalPose(), gear1, gear1->getGlobalPose());
	//joint->setLocalPose(physx::PxJointActorIndex::eACTOR0, stick1->getGlobalPose().transformInv(transform1));
	//joint->setLocalPose(physx::PxJointActorIndex::eACTOR1, anchor2->getGlobalPose().transformInv(transform2));
	//physx::PxRevoluteJoint* joint2 = physx::PxRevoluteJointCreate(*gPhysics, stick2, stick2->getGlobalPose(), gear2, gear2->getGlobalPose());
	//physx::PxRevoluteJoint* joint3 = physx::PxRevoluteJointCreate(*gPhysics, stick3, stick3->getGlobalPose(), gear3, gear3->getGlobalPose());
	//float angularVelocity = 10.0f; // radians per second
	//joint->setDriveVelocity(angularVelocity);
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
	//printf("-----------------------------------------------\n");
	//printf("Create triangles mesh kinematic actor using BVH34 midphase: \n\n");
	// Favor cooking speed, skip mesh cleanup, but precompute active edges. Insert into PxPhysics.
	// These settings are suitable for runtime cooking, although selecting more triangles per leaf may reduce
	// runtime performance of simulation and queries. We still need to ensure the triangles 
	// are valid, so we perform a validation check in debug/checked builds.
	physx::PxU32 numVertices = drawable_object[j].getModel()->meshes[i].gVertices.size();
	const physx::PxVec3* vertices = drawable_object[j].getModel()->meshes[i].gVertices.data();
	physx::PxU32 numTriangles = drawable_object[j].getModel()->meshes[i].indices.size() / 3;
	const physx::PxU32* indices = drawable_object[j].getModel()->meshes[i].indices.data();
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
	/*physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
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
	}*/

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
		actorID[meshActor] = i;
		drawable_object[j].getModel()->actorIDs.push_back(i);
		drawable_object[j].getModel()->isCharacter = true;

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
	controllerDesc.height = 2.5f;
	controllerDesc.radius = 1.0f;
	controllerDesc.position = physx::PxExtendedVec3(actor->getGlobalPose().p.x, actor->getGlobalPose().p.y, actor->getGlobalPose().p.z);
	controllerDesc.material = gMaterial;
	controllerDesc.slopeLimit = 0.5f;
	controllerDesc.contactOffset = 0.1f;
	controllerDesc.stepOffset = 0.2f;

	gController = gControllerManager->createController(controllerDesc);
	gController->setUserData(actor);
	gController->setFootPosition(physx::PxExtendedVec3(actor->getGlobalPose().p.x, actor->getGlobalPose().p.y, actor->getGlobalPose().p.z));
}

void Scene::createConvexMeshes(int i, int j)
{
	//printf("-----------------------------------------------\n");
	//printf("Create convex mesh dynamic actor: \n\n");
	// Favor cooking speed, skip mesh cleanup, but precompute active edges. Insert into PxPhysics.
	// These settings are suitable for runtime cooking, although selecting more triangles per leaf may reduce
	// runtime performance of simulation and queries. We still need to ensure the triangles 
	// are valid, so we perform a validation check in debug/checked builds.
	physx::PxU32 numVertices = drawable_object[j].getModel()->meshes[i].gVertices.size();
	const physx::PxVec3* vertices = drawable_object[j].getModel()->meshes[i].gVertices.data();
	physx::PxU32 numTriangles = drawable_object[j].getModel()->meshes[i].indices.size() / 3;
	const physx::PxU32* indices = drawable_object[j].getModel()->meshes[i].indices.data();
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
			/*physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
			float elapsedTime = physx::shdfnd::Time::getCounterFrequency().toTensOfNanos(stopTime - startTime) / (100.0f * 1000.0f);
			printf("\t -----------------------------------------------\n");
			printf("\t Create triangle mesh with %d triangles: FAILED\n", numTriangles);
			printf("\t Elapsed time in ms: %f \n", double(elapsedTime));*/

			return;
		}
		meshShape->setFlag(physx::PxShapeFlag::eSIMULATION_SHAPE, true);
		meshShape->setFlag(physx::PxShapeFlag::eSCENE_QUERY_SHAPE, true);
		
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
		
		physx::PxRigidBodyExt::updateMassAndInertia(*meshActor, 1.6f);
		gScene->addActor(*meshActor);

		actorID[meshActor] = i;
		this->num_balls = i;
		drawable_object[j].getModel()->actorIDs.push_back(i);
	}

	// Print the elapsed time for comparison
	/*physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
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
	}*/
}

void Scene::createStaticActor(int i, int j)
{
	//printf("-----------------------------------------------\n");
	//printf("Create triangles mesh static actor: \n\n");
	// Favor cooking speed, skip mesh cleanup, but precompute active edges. Insert into PxPhysics.
	// These settings are suitable for runtime cooking, although selecting more triangles per leaf may reduce
	// runtime performance of simulation and queries. We still need to ensure the triangles 
	// are valid, so we perform a validation check in debug/checked builds.
	physx::PxU32 numVertices = drawable_object[j].getModel()->meshes[i].gVertices.size();
	const physx::PxVec3* vertices = drawable_object[j].getModel()->meshes[i].gVertices.data();
	physx::PxU32 numTriangles = drawable_object[j].getModel()->meshes[i].indices.size() / 3;
	const physx::PxU32* indices = drawable_object[j].getModel()->meshes[i].indices.data();
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
	/*physx::PxU64 stopTime = physx::shdfnd::Time::getCurrentCounterValue();
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
	}*/
}

void Scene::createWheelActor(int i, int j, int* gears, int* sticks) //TODO: fixnout ty ozubena kola
{
	printf("-----------------------------------------------\n");
	printf("Create gear actor: \n\n");
	physx::PxU32 numVertices = drawable_object[j].getModel()->meshes[i].gVertices.size();
	const physx::PxVec3* vertices = drawable_object[j].getModel()->meshes[i].gVertices.data();
	physx::PxU32 numTriangles = drawable_object[j].getModel()->meshes[i].indices.size() / 3;
	const physx::PxU32* indices = drawable_object[j].getModel()->meshes[i].indices.data();

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
		actorID[meshActor] = i;
		drawable_object[j].getModel()->actorIDs.push_back(i);
		gScene->addActor(*meshActor);
	}
	if (j == drawable_object.size() - 3)
	{
		switch (*gears)
		{
		case 0:
			gear1 = meshActor;
			(*gears)++;
			break;
		case 1:
			gear2 = meshActor;
			(*gears)++;
			break;
		case 2:
			gear3 = meshActor;
			(*gears)++;
			break;
		default:
			break;
		}
	}
	else if (j == drawable_object.size() - 2)
	{

		switch (*sticks)
		{
		case 0:
			stick1 = meshActor;
			(*sticks)++;
			break;
		case 1:
			stick2 = meshActor;
			(*sticks)++;
			break;
		case 2:
			gear3 = meshActor;
			(*sticks)++;
			break;
		default:
			break;
		}
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
	if (cudaON)
	{
		gCudaContextManager->release();
	}

	printf("PhysX done.\n");
}

void Scene::applyPhysXTransform(const float delta, const physx::PxVec3 gravity)
{
	const physx::PxU32 numActors = gScene->getNbActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC);
	physx::PxActor** actors = new physx::PxActor * [numActors];
	gScene->getActors(physx::PxActorTypeFlag::eRIGID_DYNAMIC, actors, numActors);
	int characterNum = 1;

	// Iterate through actors and get their position
	for (physx::PxU32 i = 0; i < numActors; i++)
	{
		auto found = actorID.find(actors[i]);
		if (found != actorID.end())
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
				if (drawable_object[j].getModel()->isCharacter)
				{
					characterNum = j;
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
			/*
			if (meshID == 16)
			{
				std::cout << "break\n";
			}*/
			this->drawable_object[drawObjID].getModel()->applyPhysxTransf(openMatrix, meshID);
		}
	}
	//move character
	physx::PxRigidActor* character = gController->getActor();
	physx::PxTransform actorTransform = character->getGlobalPose();
	physx::PxMat44 matrix = physx::PxMat44(actorTransform);
	physx::PxVec3 actorPosition = actorTransform.p;

	glm::mat4 openMatrix = glm::mat4(matrix.column0.x, matrix.column0.y, matrix.column0.z, matrix.column0.w, matrix.column1.x, matrix.column1.y, matrix.column1.z, matrix.column1.w, matrix.column2.x, matrix.column2.y, matrix.column2.z, matrix.column2.w, matrix.column3.x, matrix.column3.y, matrix.column3.z, matrix.column3.w);

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
	switch (this->drawable_object[characterNum].getModel()->moved)
	{
	case 0:
		collisionFlags = gController->move(controllerState.deltaXP, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 1:
		forwardDir = physx::PxVec3(camera->view()[2].x, camera->view()[2].y, -camera->view()[2].z); // Extract the forward direction vector from the view matrix
		movement = forwardDir + controllerState.deltaXP;
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 2:
		forwardDir = physx::PxVec3(-camera->view()[2].x, camera->view()[2].y, camera->view()[2].z);
		movement = forwardDir + controllerState.deltaXP;
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 3:
		glm::vec3 cameraLeft = glm::normalize(glm::cross(cameraForward, cameraUp));
		movement = physx::PxVec3(-cameraLeft.x, cameraLeft.y, -cameraLeft.z) + controllerState.deltaXP; // move one unit sideways to the left
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	case 4:
		glm::vec3 cameraRight = glm::normalize(glm::cross(cameraForward, cameraUp));
		movement = physx::PxVec3(cameraRight.x, cameraRight.y, cameraRight.z) + controllerState.deltaXP; // move one unit sideways to the left
		collisionFlags = gController->move(movement, 0.01f, delta, physx::PxControllerFilters());
		break;
	default:
		collisionFlags = gController->move(controllerState.deltaXP, 0.01f, delta, physx::PxControllerFilters());
		break;
	}
	if (this->drawable_object[characterNum].getModel()->moved != this->drawable_object[characterNum].getModel()->last_moved && this->drawable_object[characterNum].getModel()->moved != 0)
	{
		//TODO tohle nefunguje, kdovi proc
		switch (this->drawable_object[characterNum].getModel()->moved)
		{
		case 1:
			switch (this->drawable_object[characterNum].getModel()->last_moved)
			{
			case 2:
				this->drawable_object[characterNum].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 3:
				this->drawable_object[characterNum].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 4:
				this->drawable_object[characterNum].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		case 2:
			switch (this->drawable_object[characterNum].getModel()->last_moved)
			{
			case 1:
				this->drawable_object[characterNum].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 3:
				this->drawable_object[characterNum].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 4:
				this->drawable_object[characterNum].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		case 3:
			switch (this->drawable_object[characterNum].getModel()->last_moved)
			{
			case 1:
				this->drawable_object[characterNum].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 2:
				this->drawable_object[characterNum].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 4:
				this->drawable_object[characterNum].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		case 4:
			switch (this->drawable_object[characterNum].getModel()->last_moved)
			{
			case 1:
				this->drawable_object[characterNum].getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 2:
				this->drawable_object[characterNum].getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			case 3:
				this->drawable_object[characterNum].getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
		this->drawable_object[characterNum].getModel()->last_moved = drawable_object[characterNum].getModel()->moved;
	}

	this->drawable_object[characterNum].getModel()->moved = 0;

	if (this->drawable_object[characterNum].getModel()->shot)
	{
		physx::PxVec3 pxCameraPos(camera->position().x, camera->position().y, camera->position().z);
		physx::PxVec3 pxCameraDir(camera->direction().x, camera->direction().y, camera->direction().z);
		float sphereRadius = 0.95f; // Set the radius of the sphere.
		physx::PxVec3 spherePos = pxCameraPos + (pxCameraDir * (sphereRadius + 1.0f)); // Add 1 unit to avoid collision with the camera.
		physx::PxTransform sphereTransform(spherePos);
		float sphereSpeed = 50.0f; // Set the speed of the sphere.
		physx::PxVec3 sphereVelocity = pxCameraDir * sphereSpeed;
		physx::PxSphereGeometry sphereGeometry(sphereRadius);
		physx::PxRigidDynamic* new_ball = shootBall(sphereTransform, sphereGeometry, sphereVelocity);
		if (ball_exist)
		{
			for (int k = 0; k < drawable_object.size(); k++)
			{
				if (drawable_object[k].getModel()->isBall)
				{
					//this->drawable_object.emplace_back(DrawableObject( new Models(*drawable_object[k].getModel()), ShaderInstances::phong(), TextureManager::getOrEmplace("sphere2", "Textures/white_tex.png"), drawable_object.size(), true, 1));
					this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere2"), ShaderInstances::phong(), TextureManager::getOrEmplace("sphere2", "Textures/white_tex.png"), drawable_object.size(), true, 1));
					break;
				}
			}
		}
		else
		{
			this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("sphere2"), ShaderInstances::phong(), TextureManager::getOrEmplace("sphere2", "Textures/white_tex.png"), drawable_object.size(), true, 1));
			this->ball_exist = true;
		}
		num_balls++;
		actorID[new_ball] = num_balls;
		drawable_object.back().getModel()->actorIDs.push_back(num_balls);
		drawable_object.back().getModel()->isBall = true;
		this->drawable_object[characterNum].getModel()->shot = false;
	}

	// Release memory for actors array
	delete[] actors;
}

physx::PxRigidDynamic* Scene::shootBall(const physx::PxTransform& t, const physx::PxGeometry& geometry, const physx::PxVec3& velocity)
{
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

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("laboratory"), ShaderInstances::phong(), TextureManager::getOrEmplace("laboratory", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(-20.0f, 0.3f, 22.0f));
	this->drawable_object.back().getModel()->rotate(270.0f, glm::vec3(0.0f, 1.0f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("pizza"), ShaderInstances::phong(), TextureManager::getOrEmplace("pizza", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(20.0f, 0.4f, 22.0f));
	this->drawable_object.back().getModel()->rotate(90.0f, glm::vec3(0.0f, 1.0f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("swiss_house"), ShaderInstances::phong(), TextureManager::getOrEmplace("swiss_house", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(25.0f, 0.3f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("bighouse"), ShaderInstances::phong(), TextureManager::getOrEmplace("bighouse", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.0f, 0.3f, -20.0f));

	//TODO: fixnout ty ozubena kola
	/*
	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("gears"), ShaderInstances::phong(), TextureManager::getOrEmplace("gears", "Textures/white_tex.png"), drawable_object.size(), true, 1));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.0f, 0.1f, -20.0f));
	this->drawable_object.back().getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("stick"), ShaderInstances::phong(), TextureManager::getOrEmplace("stick", "Textures/white_tex.png"), drawable_object.size(), true, 1));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.0f, 0.1f, -20.0f));
	this->drawable_object.back().getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));

	this->drawable_object.emplace_back(DrawableObject(ModelsLoader::get("bases"), ShaderInstances::phong(), TextureManager::getOrEmplace("bases", "Textures/white_tex.png"), drawable_object.size(), true, 0));
	this->drawable_object.back().getModel()->Pos_mov(glm::vec3(0.0f, 0.1f, -20.0f));
	this->drawable_object.back().getModel()->rotate(180.0f, glm::vec3(0.0f, 1.0f, 0.0f));
	*/
	
	camera = new Camera();
	camera->registerObserver(ShaderInstances::constant());
	camera->registerObserver(ShaderInstances::phong());
	camera->registerObserver(ShaderInstances::terrain());
	camera->registerObserver(ShaderInstances::phong_no_textures());
	camera->registerObserver(ShaderInstances::phong_norm());
	camera->registerObserver(ShaderInstances::skybox());

	emplaceLight(glm::vec3{ 0.6f }, glm::vec3{ -10.f, 50.f, -20.f }, gl::Light::Directional); //SUN OR MOON
	emplaceLight(glm::vec3{ 0.6f }, glm::vec3{ 10.f, 50.f, -20.f }, gl::Light::Directional); //SUN OR MOON
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