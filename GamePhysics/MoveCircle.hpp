#pragma once
#include "MovementCalculator.hpp"
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>

class MoveCircle : public MovementCurve
{
    const glm::vec3 center;
    const float radius;
    const float r2;

    static float calcAngle(float t);
public:
    MoveCircle(glm::vec3 center, float radius);

    glm::vec3 position(float t) const override;
    glm::vec3 direction(float t) const override;
};