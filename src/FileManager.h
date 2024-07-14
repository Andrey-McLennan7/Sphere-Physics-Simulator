#pragma once

#ifndef _FILE_MANAGER_H_
#define _FILE_MANAGER_H_

#include <fstream>
#include <vector>
#include <string>

class DynamicObject;
class CollisionPlane;

class FileManager
{
private:

	std::ifstream inputFile;
	bool success;

	void inputVec3(std::stringstream* inputString, glm::vec3& vector);
	void inputFloat(std::stringstream* inputString, float& scalar);

	float randomColour();

public:

	FileManager(const std::string& fileName);

	void loadData(std::vector<DynamicObject>& objects);
	void loadColisionPlanes(std::vector<DynamicObject>& objects, CollisionPlane* plane);
	void loadColisionObjects(std::vector<DynamicObject>& objects);

	bool status() const { return this->success; }
};

#endif