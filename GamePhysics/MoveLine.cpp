#include "MoveLine.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

static glm::vec3 operator*(const glm::vec3 vec, const float f) 
{
    return { vec.x * f, vec.y * f, vec.z * f };
}

glm::vec3 MoveLine::calcDir(const glm::vec3 begin, const glm::vec3 end)
{
    glm::vec3 dir = glm::vec3{ end.x, 0.f, end.z } - glm::vec3{ begin.x, 0.f, begin.z };
    dir = glm::normalize(dir);

    float angle = std::acos(glm::dot(dir, glm::vec3{ 1.f, 0.f, 0.f }));
    if (dir.z < 0.f) 
    {
        angle = 2 * M_PI - angle;
    }

    return { 0.f, -angle, 0.f };
}

MoveLine::MoveLine(glm::vec3 begin, glm::vec3 end) : begin(begin), end(end), dir(calcDir(begin, end))
{
}

glm::vec3 MoveLine::position(float t) const
{
    return begin * (1.f - t) + end * t;
}

glm::vec3 MoveLine::direction(float t) const
{
    (void)t;
    return dir;
}