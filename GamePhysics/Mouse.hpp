#pragma once
#include <functional>
#include <cstdint>
#include <vector>
#include <memory>
#include <algorithm>

#include "Observer.hpp"

struct MouseData {
	int x;
	int y;
	int dx;
	int dy;
	uint8_t buttons;

	bool lbPressed() const;
	bool mbPressed() const;
	bool rbPressed() const;

	MouseData();
	MouseData(int x, int y, int dx, int dy, uint8_t buttons);
};

class Mouse : public Observable
{
private:
	static Mouse* mouse;

	int x = 0;
	int y = 0;
	uint8_t pressed = 0;
	MouseData current;

	Mouse();
public:
	enum class Button {
		LB = 0b1,
		MB = 0b10,
		RB = 0b100,
		Other = 0b1000
	};

	void buttonPress(Button button);
	void buttonRelease(Button button);
	void mouseMove(const int nx, const int ny);

	const MouseData& data() const;

	static Mouse& instance();
};