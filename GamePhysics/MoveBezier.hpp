#pragma once
#include "MovementCalculator.hpp"
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
#include <functional>

class MoveBezier : public MovementCurve
{
    const std::function<glm::vec3(float)> bezierCallback;
public:
    MoveBezier(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4);

    glm::vec3 position(float t) const override;
    glm::vec3 direction(float t) const override;
};