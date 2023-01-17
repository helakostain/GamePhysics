#include "Light.hpp"

static float boundColor(const float color) 
{
	return std::max(std::min(1.f, color), 0.f);
}

static glm::vec3 boundColor(const glm::vec3 color) 
{
	return { boundColor(color.x), boundColor(color.y), boundColor(color.z) };
}

ColoredLight::ColoredLight() = default;

ColoredLight::ColoredLight(const glm::vec3 color) : color(boundColor(color)) 
{
}

gl::Light ColoredLight::type() const
{
	return gl::Light::Ambient;
}

void ColoredLight::update(const glm::vec3 newDirection, const glm::vec3 newPos)
{
}

void ColoredLight::apply() const
{
	notifyObservers(EventType::LightChanged, (void*)this);
}

void ColoredLight::addColor(glm::vec3 delta) const 
{
	setColor(color + delta);
}

void ColoredLight::setColor(const glm::vec3 newColor) const 
{
	color = boundColor(newColor);
	apply();
}

glm::vec3 ColoredLight::getColor() const
{
	return color;
}

void ColoredLight::setCutoff(float cutoff) const
{
}

float ColoredLight::getCutoff() const
{
	return 0.0f;
}

PositionedLight::PositionedLight() : ColoredLight() 
{
}

PositionedLight::PositionedLight(const glm::vec3 color, const glm::vec3 position) : ColoredLight(color), position(position) 
{
}

gl::Light PositionedLight::type() const
{
	return gl::Light::Point;
}

void PositionedLight::move(glm::vec3 delta) const
{
	setPosition(position + delta);
}

void PositionedLight::setPosition(glm::vec3 newPos) const
{
	position = newPos;
	apply();
}

glm::vec3 PositionedLight::getPosition() const
{
	return position;
}

DirectionalLight::DirectionalLight() : ColoredLight() 
{
}

DirectionalLight::DirectionalLight(glm::vec3 color, glm::vec3 direction) : ColoredLight(color), direction(direction)
{
}

gl::Light DirectionalLight::type() const
{
	return gl::Light::Directional;
}

void DirectionalLight::setDirection(glm::vec3 newDirection) const
{
	direction = newDirection;
	apply();
}

glm::vec3 DirectionalLight::getDirection() const
{
	return direction;
}

Spotlight::Spotlight() : PositionedLight() 
{
}

Spotlight::Spotlight(glm::vec3 color, glm::vec3 position, glm::vec3 direction, float cutoff) : PositionedLight(color, position), direction(direction), cutoff(cutoff)
{
}

gl::Light Spotlight::type() const
{
	return gl::Light::Spotlight;
}

void Spotlight::update(const glm::vec3 newDirection, const glm::vec3 newPosition)
{
	this->setDirection(newDirection);
	this->setPosition(newPosition);
	apply();
}

void Spotlight::setDirection(glm::vec3 newDirection) const
{
	direction = newDirection;
	apply();
}

glm::vec3 Spotlight::getDirection() const
{
	return direction;
}

void Spotlight::setCutoff(float newCutoff) const
{
	cutoff = newCutoff;
	apply();
}

float Spotlight::getCutoff() const
{
	return cutoff;
}