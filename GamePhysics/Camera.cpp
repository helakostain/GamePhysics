#include "Camera.hpp"

void Camera::updateCameraMatrix()
{
	camera = glm::lookAt(eye, eye + target, up);
}

void Camera::calcTarget()
{
	target = glm::vec3
	{
		std::cos(fi),
		std::sin(psi),
		std::sin(fi)
	};
}

void Camera::capAngles()
{
	fi = std::fmod(fi, 360.f);
	psi = std::max(psi, -85.f);
	psi = std::min(psi, 85.f);
}

void Camera::updateAngle(const float dt)
{
	const float dFi = hRotate * dt * rotSpeed;
	const float dPsi = vRotate * dt * rotSpeed;

	fi += dFi;
	psi += dPsi;

	capAngles();

	changeMade = changeMade || dFi || dPsi;
}

void Camera::updateForwardMovement(const float dt)
{
	const float sideways = sidewaysMovement * dt * movSpeed;
	const float forward = forwardMovement * dt * movSpeed;

	const float dx = std::cos(fi) * forward + std::cos(fi + M_PI_2) * sideways;
	const float dy = std::sin(psi) * forward;
	const float dz = std::sin(fi) * forward + std::sin(fi + M_PI_2) * sideways;

	eye.x += dx;
	eye.y += dy;
	eye.z += dz;

	changeMade = changeMade || dx || dy || dz;
}

Camera::Camera()
{
	updateCameraMatrix();
}

void Camera::setPosition(glm::vec3 pos)
{
	eye = pos;
	calcTarget();
	updateCameraMatrix();
	apply();
}

void Camera::moveSideways(Direction dir)
{
	sidewaysMovement = (int)dir;
}

void Camera::moveForward(Direction dir)
{
	forwardMovement = (int)dir;
}

void Camera::rotateHor(Direction dir)
{
	hRotate = (int)dir;
}

void Camera::rotateVer(Direction dir)
{
	vRotate = (int)dir;
}

void Camera::update(const float dt)
{
	updateAngle(dt);
	updateForwardMovement(dt);
	calcTarget();
	updateCameraMatrix();

	if (changeMade)
	{
		apply();
		changeMade = false;
	}
}

void Camera::apply()
{
	notifyObservers(EventType::CameraMoved, this);
}

void Camera::onMouseMove(const MouseData& md)
{
	if (not md.lbPressed())
	{
		return;
	}

	const float rad_x = md.dx / 180.f * M_PI;
	const float rad_y = md.dy / 180.f * M_PI;

	const float dFi = rad_x * dragSpeed;
	const float dPsi = -1.f * rad_y * dragSpeed;

	fi += dFi;
	psi += dPsi;

	capAngles();

	changeMade = dFi || dPsi;
}

void Camera::WindowChange(int width, int height)
{
	projection = glm::perspective(glm::radians(45.f), (float)width / (float)height, 0.1f, 100.f);
	apply();
}

void Camera::notify(EventType eventType, void* object)
{
	if (eventType == EventType::MouseMoved)
	{
		onMouseMove(((Mouse*)object)->data());
	}
}

glm::mat4 Camera::view() const
{
	return camera;
}

glm::mat4 Camera::project() const
{
	return projection;
}

glm::vec3 Camera::position() const
{
	return eye;
}

glm::vec3 Camera::direction() const
{
	return this->target;
}