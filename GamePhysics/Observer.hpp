#pragma once
#include <vector>

enum class EventType {
	MouseButtonPressed,
	MouseButtonReleased,
	MouseMoved,
	LightChanged,
	CameraMoved,
	PhysxChanged
};

struct Observer {
	virtual void notify(EventType eventType, void* object) = 0;
};

class Observable {
private:
	mutable std::vector<Observer*> observers;
public:
	void registerObserver(Observer& obs);
	void removeObserver(Observer& obs);
	void notifyObservers(EventType type, void* obj) const;
};