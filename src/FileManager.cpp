#include "DynamicObject.h"
#include "CollisionPlane.h"
#include "FileManager.h"

#include <sstream>
#include <iostream>

FileManager::FileManager(const std::string& fileName) :
	success{ true }
{
	this->inputFile.open(fileName);

	if (!this->inputFile)
	{
		std::cerr << "ERROR:FAILED TO READ FILE" << std::endl;
		this->success = false;
		return;
	}
}

void FileManager::loadData(std::vector<DynamicObject>& objects)
{
	std::string line;
	std::getline(this->inputFile, line);
	line = "";

	while (std::getline(this->inputFile, line))
	{
		std::stringstream inputString{ line };

		glm::vec3 position;
		glm::vec3 velocity;
		glm::vec3 acceleration;

		float scale{ 0.0f };
		float radius{ 0.0f };
		float mass{ 0.0f };
		float elasticity{ 0.0f };

		this->inputVec3(&inputString, position);
		this->inputVec3(&inputString, velocity);
		this->inputVec3(&inputString, acceleration);

		this->inputFloat(&inputString, scale);
		this->inputFloat(&inputString, radius);
		this->inputFloat(&inputString, mass);
		this->inputFloat(&inputString, elasticity);

		DynamicObject obj;

		obj.setType(PHY_SPHERE);
		obj.setColor(this->randomColour(), this->randomColour(), this->randomColour());

		obj.setPosition(position);
		obj.setVelocity(velocity);
		obj.setAcceleration(acceleration);
		obj.setScale(scale, scale, scale);
		obj.setBoundingRadius(radius);
		obj.setMass(mass);
		obj.setElasticity(elasticity);

		objects.push_back(obj);
	}

	this->inputFile.close();
}

void FileManager::loadColisionPlanes(std::vector<DynamicObject>& objects, CollisionPlane* plane)
{
	for (int i{ 0 }; i < objects.size(); ++i)
	{
		objects.at(i).setCollisionPlane(plane);
	}
}

void FileManager::loadColisionObjects(std::vector<DynamicObject>& objects)
{
	for (int i{ 0 }; i < objects.size(); ++i)
	{
		for (int j{ 0 }; j < objects.size(); ++j)
		{
			if (i == j) continue;

			objects.at(i).setCollisionObject(&objects.at(j));
		}
	}
}

void FileManager::inputVec3(std::stringstream* inputString, glm::vec3& vector)
{
	std::string tempString[3];

	for (int i{ 0 }; i < 3; ++i)
	{
		std::getline(*inputString, tempString[i], ',');
		vector[i] = atof(tempString[i].c_str());
	}
}

void FileManager::inputFloat(std::stringstream* inputString, float& scalar)
{
	std::string tempString;
	std::getline(*inputString, tempString, ',');
	scalar = atof(tempString.c_str());
}

float FileManager::randomColour()
{
	return (float)(rand() % 256) / 255.0f;
}