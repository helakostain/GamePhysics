#include <stdio.h>
#include <iostream>
#include "Application.hpp"

int main(void)
{
	Application::getInstance().Run();
	exit(EXIT_SUCCESS);
}