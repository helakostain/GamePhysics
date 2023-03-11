#define _USE_MATH_DEFINES
#include <cmath>
#include "Transformation.hpp"
#include <glm/ext/matrix_transform.hpp>

float Transformation::capV(float current, float max)
{
	return std::min(std::max(current, -max), max);
}

float Transformation::calcNewV(float current, const float acc, const float dec)
{
	if (std::abs(current) <= dec)
	{
		current = 0;
	}
	else
	{
		current -= (-1 + (2 * (current > 0))) * dec;
	}
	return current + acc;
}

float Transformation::getAcc(Direction dir)
{
	return acceleration * (int)dir;
}

void Transformation::rotate(float degree, const glm::vec3 axis)
{
	float rad = degree * M_PI / 180;
	transform = glm::rotate(transform, rad, axis);
}

void Transformation::setPosition(glm::vec3 position)
{
	transform[3] = glm::vec4(position, 1);
}

void Transformation::setMatrix(glm::mat4 newMatrix)
{
	this->transform = newMatrix;
}

void Transformation::scale(const glm::vec3 scales)
{
	transform = glm::scale(transform, scales);
}

void Transformation::updateForces(const float dt)
{
	const float dec = deceleration * dt;
	const float acc = acceleration * dt;
	const float fx = capV(calcNewV(forces.x, acc * movementDir.x, dec), maxV);
	const float fy = capV(calcNewV(forces.y, acc * movementDir.y, dec), maxV);
	forces = { fx, fy };

	translate({ fx * dt, fy * dt, 0.f });
}

void Transformation::updateRotation(const float dt)
{
	const float dec = rotationDec * dt;
	const float acc = rotationAcc * dt * (int)rotationDir;
	rotationV = capV(calcNewV(rotationV, acc, dec), rotationMax);

	rotate(rotationV * dt, glm::vec3(0.f, 0.f, 1.f));
}

void Transformation::updateGrowth(const float dt)
{
	const auto g = 1 + growthV * (int)growthDir * dt;
	scale(glm::vec3(g));
}

Transformation::Transformation()
{
}

const glm::mat4& Transformation::matrix() const
{
	return this->transform;
}

void Transformation::translate(const glm::vec3 delta)
{
	transform = glm::translate(transform, delta);
}

void Transformation::Update(const float deltaT)
{
	updateGrowth(deltaT);
	updateRotation(deltaT);
	updateForces(deltaT);
}

void Transformation::scale(float g)
{
	transform = glm::scale(transform, glm::vec3(g));
}

void Transformation::applyForce(Direction dx, Direction dy)
{
	applyFx(dx);
	applyFy(dy);
}

void Transformation::applyFx(Direction dir)
{
	this->movementDir.x = getAcc(dir);
}

void Transformation::applyFy(Direction dir)
{
	this->movementDir.y = getAcc(dir);
}

void Transformation::setGrowth(Growth direction)
{
	this->growthDir = direction;
}

void Transformation::setRotation(Rotation direction)
{
	this->rotationDir = direction;
}