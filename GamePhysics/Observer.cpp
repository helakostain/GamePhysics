#include "Observer.hpp"

void Observable::registerObserver(Observer& obs)
{
	observers.emplace_back(&obs);
}

void Observable::removeObserver(Observer& obs)
{
	auto iter = observers.begin();
	while (iter != observers.end())
	{
		if (*iter == &obs)
		{
			observers.erase(iter);
			return;
		}
		++iter;
	}
}

void Observable::notifyObservers(EventType type, void* obj) const
{
	for (Observer* obs : observers)
	{
		obs->notify(type, obj);
	}
}