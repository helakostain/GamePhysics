#include "Mouse.hpp"

Mouse* Mouse::mouse = nullptr;

bool MouseData::lbPressed() const
{
	return buttons & (uint8_t)Mouse::Button::LB;
}

bool MouseData::mbPressed() const
{
	return buttons & (uint8_t)Mouse::Button::MB;
}

bool MouseData::rbPressed() const
{
	return buttons & (uint8_t)Mouse::Button::RB;
}

MouseData::MouseData() :
	x(0), y(0), dx(0), dy(0), buttons(0) {}

MouseData::MouseData(const int x, const int y, const int dx, const int dy, const uint8_t buttons) :
	x(x), y(y), dx(dx), dy(dy), buttons(buttons) {}

Mouse::Mouse() = default;

void Mouse::buttonPress(const Button button)
{
	pressed |= (uint8_t)button;
	current = MouseData(x, y, 0, 0, pressed);

	notifyObservers(EventType::MouseButtonPressed, this);
}

void Mouse::buttonRelease(const Button button)
{
	pressed &= ~(uint8_t)button;
	current = MouseData(x, y, 0, 0, pressed);

	notifyObservers(EventType::MouseButtonReleased, this);
}

void Mouse::mouseMove(const int nx, const int ny)
{
	const int dx = nx - x;
	const int dy = ny - y;
	x = nx;
	y = ny;
	current = MouseData(x, y, dx, dy, pressed);

	notifyObservers(EventType::MouseMoved, this);
}

const MouseData& Mouse::data() const
{
	return current;
}

Mouse& Mouse::instance()
{
	if (not mouse) 
	{
		mouse = new Mouse();
	}
	return *mouse;
}