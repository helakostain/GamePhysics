#include "MoveBezier.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

static glm::vec3 operator*(const glm::vec3 vec, const float f) 
{
    return { vec.x * f, vec.y * f, vec.z * f };
}

static auto createCallback(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4)
{
    return [=](const float t) -> glm::vec3 
    {
        const glm::vec3 point1 = p1 * std::pow(1 - t, 3.f);
        const glm::vec3 point2 = p2 * 3 * t * std::pow(1 - t, 2.f);
        const glm::vec3 point3 = p3 * 3 * t * t * (1 - t);
        const glm::vec3 point4 = p4 * t * t * t;
        return point1 + point2 + point3 + point4;
    };
}

MoveBezier::MoveBezier(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4) : bezierCallback(createCallback(p1, p2, p3, p4))
{
}

glm::vec3 MoveBezier::position(float t) const
{
    return bezierCallback(t);
}

glm::vec3 MoveBezier::direction(float t) const
{
    return glm::vec3();
}