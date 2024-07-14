#include "phy.h"
#include "glm/glm.hpp"

#include "DynamicObject.h"
#include "CollisionPlane.h"
#include "FileManager.h"

#include <iostream>
#include <exception>
#include <ctime>
#include <string>

int main()
{
	srand(time(0));

	std::string fileName;

	std::cout << "File:";
	std::cin >> fileName;

	FileManager file{ fileName };
	if (!file.status())
	{
		throw std::exception();
	}

	fileName.clear();

	CollisionPlane bottom;
	CollisionPlane right;
	CollisionPlane left;
	CollisionPlane back;
	CollisionPlane front;

	bottom.setType(PHY_SQUARE);
	bottom.setColor(1.0f, 1.0f, 1.0f);
	bottom.setScale(100.0f, 100.0f, 100.0f);
	bottom.setPosition(-20.0f, 0.01f, 0.0f);
	bottom.setRotation(-90.0f, 0.0f, 0.0f);
	bottom.setMass(100000000000000000000000000.0f);
	bottom.setNormal(glm::vec3(0.0f, 1.0f, 0.0f));
	bottom.setPoint(glm::vec3(0.0f));
	bottom.setVelocity(glm::vec3(0.0f));

	right.setType(PHY_SQUARE);
	right.setColor(1.0f, 1.0f, 1.0f);
	right.setScale(100.0f, 100.0f, 100.0f);
	right.setPosition(30.0f, 0.0f, 0.0f);
	right.setRotation(0.0f, -90.0f, 0.0f);
	right.setMass(100000000000000000000000000.0f);
	right.setNormal(glm::vec3(-1.0f, 0.0f, 0.0f));
	right.setPoint(glm::vec3(30.0f, 0.0f, 0.0f));
	right.setVelocity(glm::vec3(0.0f));

	left.setType(PHY_SQUARE);
	left.setColor(1.0f, 1.0f, 1.0f);
	left.setScale(100.0f, 100.0f, 100.0f);
	left.setPosition(-70.0f, 0.0f, 0.0f);
	left.setRotation(0.0f, -90.0f, 0.0f);
	left.setMass(100000000000000000000000000.0f);
	left.setNormal(glm::vec3(1.0f, 0.0f, 0.0f));
	left.setPoint(glm::vec3(-70.0f, 0.0f, 0.0f));
	left.setVelocity(glm::vec3(0.0f));

	back.setType(PHY_SQUARE);
	back.setColor(1.0f, 1.0f, 1.0f);
	back.setScale(100.0f, 100.0f, 100.0f);
	back.setPosition(-20.0f, 0.0f, -50.0f);
	back.setRotation(0.0f, 0.0f, 0.0f);
	back.setMass(100000000000000000000000000.0f);
	back.setNormal(glm::vec3(0.0f, 0.0f, 1.0f));
	back.setPoint(glm::vec3(-20.0f, 0.0f, -50.0f));
	back.setVelocity(glm::vec3(0.0f));

	front.setType(PHY_SQUARE);
	front.setColor(1.0f, 1.0f, 1.0f);
	front.setScale(100.0f, 100.0f, 100.0f);
	front.setPosition(-20.0f, 0.0f, 50.0f);
	front.setRotation(0.0f, 0.0f, 0.0f);
	front.setMass(100000000000000000000000000.0f);
	front.setNormal(glm::vec3(0.0f, 0.0f, -1.0f));
	front.setPoint(glm::vec3(-20.0f, 0.0f, 50.0f));
	front.setVelocity(glm::vec3(0.0f));

	std::vector<DynamicObject> objects;

	float dt = 1.0f / 60.0f;

	file.loadData(objects);
	file.loadColisionObjects(objects);

	file.loadColisionPlanes(objects, &bottom);
	file.loadColisionPlanes(objects, &right);
	file.loadColisionPlanes(objects, &left);
	file.loadColisionPlanes(objects, &back);
	file.loadColisionPlanes(objects, &front);

	while (true)
	{
		for (int i{ 0 }; i < objects.size(); ++i)
		{
			objects.at(i).update(dt);
		}

		wait(5);
	}

	return 0;
}
