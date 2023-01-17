#include "MovementCalculator.hpp"
#include "MoveLine.hpp"
#include "MoveCircle.hpp"
#include "MoveBezier.hpp"
#include <iostream>

float MovementCalculator::currentT() const
{
    return t / duration;
}

MovementCalculator::MovementCalculator(std::shared_ptr<MovementCurve> curve, glm::vec3 rotationOffset, float duration) :
    curve(std::move(curve)), offset(rotationOffset), duration(duration) { }

void MovementCalculator::update(float dt)
{
    t = std::fmod(t + dt, duration);
}

glm::vec3 MovementCalculator::currentPosition() const
{
    return curve->position(currentT());
}

glm::vec3 MovementCalculator::currentHeading() const
{
    return curve->direction(currentT()) + offset;
}