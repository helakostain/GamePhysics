#pragma once
#define _USE_MATH_DEFINES

#include <glm/mat4x4.hpp>
#include <glm/ext/matrix_float4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>

#include <math.h>
#include <functional>
#include <vector>

#include "Mouse.hpp"
#include "Transformation.hpp"
#include "Observer.hpp"

class Camera : public Observer, public Observable
{
private:
	glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)16.f / 9.f, 0.1f, 100.0f); // 16:9 aspect ratio
	glm::mat4 camera;

	glm::vec3 eye{ 0.f, 5.f, -8.f };
	glm::vec3 target{ 0.f, 0.f, 0.f };
	glm::vec3 up{ 0.f, 1.f, 0.f };

	float fi = -1.5f * float(M_PI);
	float psi = 0.f;
	int sidewaysMovement = 0;
	int forwardMovement = 0;
	int hRotate = 0;
	int vRotate = 0;
	bool changeMade = false;

	static constexpr float movSpeed = 3.f;
	static constexpr float rotSpeed = 1.f;
	static constexpr float dragSpeed = 0.3f;

	void updateCameraMatrix();
	void calcTarget();
	void capAngles();
	void updateAngle(float dt);
	void updateForwardMovement(float dt);
public:
	Camera();

	void setPosition(glm::vec3 pos);
	void moveSideways(Direction dir);
	void moveForward(Direction dir);
	void rotateHor(Direction dir);
	void rotateVer(Direction dir);
	void update(float dt);
	void apply();
	void onMouseMove(const MouseData& md);
	void WindowChange(int width, int height);
	void setCamera(glm::mat4 cam);

	void notify(EventType eventType, void* object) override;

	glm::mat4 view() const;
	glm::mat4 project() const;
	glm::vec3 position() const;
	glm::vec3 direction() const;
	glm::vec3 getUp() const;
};