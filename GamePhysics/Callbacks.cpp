#include "Callbacks.hpp"

double Callbacks::clickX = 0;
double Callbacks::clickY = 0;
std::vector<DrawableObject> Callbacks::drawableObj = std::vector<DrawableObject>{};
Camera* Callbacks::camera = nullptr;
std::shared_ptr<ColoredLight> Callbacks::flashlight = nullptr;
int model_id = -1;

void Callbacks::error_callback(int error, const char* description)
{
    printf("This is error callback!\n");
    fputs(description, stderr);
}

void Callbacks::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE and action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
    int characterNum = 1;
    for (int i = 0; i < drawableObj.size(); i++)
    {
        if (drawableObj[i].getModel()->isCharacter)
        {
            characterNum = i;
            break;
        }
    }

    switch (key) {
    case GLFW_KEY_I:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setFy((action == GLFW_RELEASE) ? Direction::none : Direction::up);
        }
        break;
    case GLFW_KEY_K:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setFy((action == GLFW_RELEASE) ? Direction::none : Direction::down);
        }
        break;
    case GLFW_KEY_J:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setFx((action == GLFW_RELEASE) ? Direction::none : Direction::left);
        }
        break;
    case GLFW_KEY_L:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setFx((action == GLFW_RELEASE) ? Direction::none : Direction::right);
        }
        break;
    case GLFW_KEY_W:
        drawableObj[characterNum].getModel()->moved = 1;
        //camera->moveForward((action == GLFW_RELEASE) ? Direction::none : Direction::up);
        break;
    case GLFW_KEY_S:
        drawableObj[characterNum].getModel()->moved = 2;
        //camera->moveForward((action == GLFW_RELEASE) ? Direction::none : Direction::down);
        break;
    case GLFW_KEY_A:
        drawableObj[characterNum].getModel()->moved = 3;
        //camera->moveSideways((action == GLFW_RELEASE) ? Direction::none : Direction::left);
        break;
    case GLFW_KEY_D:
        drawableObj[characterNum].getModel()->moved = 4;
        //camera->moveSideways((action == GLFW_RELEASE) ? Direction::none : Direction::right);
        break;
    case GLFW_KEY_E:
        if (Callbacks::flashlight->getCutoff() != 0 && action == GLFW_PRESS)
            Callbacks::flashlight->setCutoff(0);
        else if (action == GLFW_PRESS)
            Callbacks::flashlight->setCutoff(9.f);
        break;
    case GLFW_KEY_UP:
        drawableObj[characterNum].getModel()->moved = 1;
        //camera->rotateVer((action == GLFW_RELEASE) ? Direction::none : Direction::up);
        break;
    case GLFW_KEY_DOWN:
        drawableObj[characterNum].getModel()->moved = 2;
        //camera->rotateVer((action == GLFW_RELEASE) ? Direction::none : Direction::down);
        break;
    case GLFW_KEY_LEFT:
        drawableObj[characterNum].getModel()->moved = 3;
        //camera->rotateHor((action == GLFW_RELEASE) ? Direction::none : Direction::left);
        break;
    case GLFW_KEY_RIGHT:
        drawableObj[characterNum].getModel()->moved = 4; 
        //camera->rotateHor((action == GLFW_RELEASE) ? Direction::none : Direction::right);
        break;
    case GLFW_KEY_U:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setRot((action == GLFW_RELEASE) ? Rotation::none : Rotation::left);
        }
        break;
    case GLFW_KEY_O:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setRot((action == GLFW_RELEASE) ? Rotation::none : Rotation::right);
        }
        break;
    case GLFW_KEY_KP_ADD:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setGrow((action == GLFW_RELEASE) ? Growth::none : Growth::grow);
        }
        break;
    case GLFW_KEY_KP_SUBTRACT:
        if (model_id > 1)
        {
            drawableObj[model_id].getModel()->setGrow((action == GLFW_RELEASE) ? Growth::none : Growth::shrink);
        }
        break;
    case GLFW_KEY_SPACE:
        drawableObj[characterNum].getModel()->shot = ((action == GLFW_RELEASE) ? true : false); 
        break;
    default:
        break;
    }
}

void Callbacks::cursor_callback(GLFWwindow* window, double x, double y)
{
    Mouse::instance().mouseMove((int)x, (int)y);
}

void Callbacks::button_callback(GLFWwindow* window, int button, int action, int mode)
{
    if (action == GLFW_PRESS)
    {
        printf("button_callback [%d,%d,%d]\n", button, action, mode);
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            glfwGetCursorPos(window, &clickX, &clickY);
        }
    }
}

void Callbacks::window_focus_callback(GLFWwindow* window, int focused)
{
    printf("window_focus_callback \n");
}

void Callbacks::window_iconify_callback(GLFWwindow* window, int iconified)
{
    printf("window_iconify_callback \n");
}

void Callbacks::window_size_callback(GLFWwindow* window, int width, int height)
{
    printf("resize %d, %d \n", width, height);
    glViewport(0, 0, width, height);

    Application::getInstance().height = height;
    Application::getInstance().width = width;

    Callbacks::camera->WindowChange(width, height);
}

Callbacks::Callbacks()
{
}

Callbacks::~Callbacks()
{
}

void Callbacks::Init(GLFWwindow* window, std::vector<DrawableObject>& dO, Camera* cam, shared_ptr<ColoredLight> new_flashlight)
{
    Callbacks::camera = cam;
    Callbacks::drawableObj = dO;
    Callbacks::flashlight = new_flashlight;

    auto mouseButton = [](GLFWwindow* win, int button, int action, int mode) {
        Mouse::Button btn = Mouse::Button::Other;
        if (button == GLFW_MOUSE_BUTTON_1) {
            btn = Mouse::Button::LB;
        }
        else if (button == GLFW_MOUSE_BUTTON_2) {
            btn = Mouse::Button::MB;
        }
        else if (button == GLFW_MOUSE_BUTTON_3) {
            btn = Mouse::Button::RB;
        }
        if (action == GLFW_PRESS) {
            Mouse::instance().buttonPress(btn);
        }
        else if (action == GLFW_RELEASE) {
            Mouse::instance().buttonRelease(btn);
        }
    };

    glfwSetErrorCallback(Callbacks::error_callback);
    glfwSetKeyCallback(window, Callbacks::key_callback);
    glfwSetCursorPosCallback(window, Callbacks::cursor_callback);
    glfwSetMouseButtonCallback(window, mouseButton);
    glfwSetWindowFocusCallback(window, Callbacks::window_focus_callback);
    glfwSetWindowIconifyCallback(window, Callbacks::window_iconify_callback);
    glfwSetWindowSizeCallback(window, Callbacks::window_size_callback);
}

void Callbacks::Init(GLFWwindow* window, std::vector<DrawableObject>& dO, Camera* camera)
{
    Init(window, dO, camera, nullptr);
}

void Callbacks::setObject(int id)
{
    model_id = id - 1;
}

void Callbacks::updateObjects(std::vector<DrawableObject>& dObjects)
{
    drawableObj = dObjects;
}