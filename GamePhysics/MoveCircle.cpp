#include "MoveCircle.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <memory>
#include <functional>

float MoveCircle::calcAngle(float t)
{
    constexpr float twopi = 2.f * M_PI;
    return twopi * t;
}

MoveCircle::MoveCircle(glm::vec3 center, float radius) : center(center), radius(radius), r2(radius* radius) 
{
}

glm::vec3 MoveCircle::position(float t) const
{
    const float angle = calcAngle(t);
    const float x = center.x + std::cos(angle) * radius;
    const float y = center.y;
    const float z = center.z + std::sin(angle) * radius;
    return { x, y, z };
}

glm::vec3 MoveCircle::direction(float t) const
{
    const float angle = calcAngle(t);
    return { 0.f, -angle - M_PI_2, 0.f };
}