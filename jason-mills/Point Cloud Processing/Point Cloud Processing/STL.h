#pragma once
#include "STL_Struct.h"
#include <cstdint>
#include <vector>
#include <string>

class STL
{
private:
	std::string filepath;
	uint8_t header[80];
	uint32_t triangleCount;
	std::vector<stlTriangle> triangles;
	void readFile();
	float xMax, yMax, zMax;
	float xMin, yMin, zMin;
public:
	STL(std::string filepath);

	std::string getName() {return reinterpret_cast<char*>(header);}
	uint32_t getTriangleCount() { return triangleCount; }
	std::vector<stlTriangle> getTriangles() { return triangles; }
	void normalize();
};