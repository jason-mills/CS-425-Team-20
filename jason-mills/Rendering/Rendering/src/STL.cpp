#include "STL.h"
#include <fstream>
#include <iostream>

STL::STL(std::string filepath) 
	: filepath(filepath),
	xMax(-1e+30),
	xMin(1e+30),
	yMax(-1e+30),
	yMin(1e+30),
	zMax(-1e+30),
	zMin(1e+30)
{
	readFile();
	std::cout << "X Max, X Min: " << xMax << ", " << xMin << std::endl;
	std::cout << "Y Max, Y Min: " << yMax << ", " << yMin << std::endl;
	std::cout << "Z Max, Z Min: " << zMax << ", " << zMin << std::endl;
	if (xMax > 1 || yMax > 1 || zMax > 1 || xMin < -1 || yMin < -1 || zMin < -1)
		normalize();
}

void STL::readFile() 
{
	std::ifstream file;
	stlTriangle temp;

	file.open(filepath, std::ios::binary);
	if (!file.good())
	{
		std::cout << "Could not open file." << std::endl;
		return;
	}

	file.read(reinterpret_cast<char*>(&header), sizeof(header));
	file.read(reinterpret_cast<char*>(&triangleCount), sizeof(triangleCount));

	for (int i = 0; i < triangleCount; i++) 
	{
		file.read(reinterpret_cast<char*>(&temp.normalVector), sizeof(temp.normalVector));
		file.read(reinterpret_cast<char*>(&temp.vertex1), sizeof(temp.vertex1));
		file.read(reinterpret_cast<char*>(&temp.vertex2), sizeof(temp.vertex2));
		file.read(reinterpret_cast<char*>(&temp.vertex3), sizeof(temp.vertex3));
		file.read(reinterpret_cast<char*>(&temp.color), sizeof(temp.color));

		if (temp.vertex1[0] < xMin) xMin = temp.vertex1[0];
		if (temp.vertex1[1] < yMin) yMin = temp.vertex1[1];
		if (temp.vertex1[2] < zMin) zMin = temp.vertex1[2];
		if (temp.vertex1[0] > xMax) xMax = temp.vertex1[0];
		if (temp.vertex1[1] > yMax) yMax = temp.vertex1[1];
		if (temp.vertex1[2] > zMax) zMax = temp.vertex1[2];

		if (temp.vertex2[0] < xMin) xMin = temp.vertex2[0];
		if (temp.vertex2[1] < yMin) yMin = temp.vertex2[1];
		if (temp.vertex2[2] < zMin) zMin = temp.vertex2[2];
		if (temp.vertex2[0] > xMax) xMax = temp.vertex2[0];
		if (temp.vertex2[1] > yMax) yMax = temp.vertex2[1];
		if (temp.vertex2[2] > zMax) zMax = temp.vertex2[2];

		if (temp.vertex3[0] < xMin) xMin = temp.vertex3[0];
		if (temp.vertex3[1] < yMin) yMin = temp.vertex3[1];
		if (temp.vertex3[2] < zMin) zMin = temp.vertex3[2];
		if (temp.vertex3[0] > xMax) xMax = temp.vertex3[0];
		if (temp.vertex3[1] > yMax) yMax = temp.vertex3[1];
		if (temp.vertex3[2] > zMax) zMax = temp.vertex3[2];
		triangles.push_back(temp);
	}

	file.close();

	return;
}

void STL::normalize() {
	//Doing this because I want to use basic triangle method first, might be able to let OpenGL normalize it for me later
	//Center range around 0 then divide to get it between -1 and 1
	float xAverage = (xMin + xMax) / 2;
	float xRange = (xMax - xMin) / 2;
	float yAverage = (yMin + yMax) / 2;
	float yRange = (yMax - yMin) / 2;
	float zAverage = (zMin + zMax) / 2;
	float zRange = (zMax - zMin) / 2;

	for (int i = 0; i < triangleCount; i++) {
		triangles[i].vertex1[0] = (triangles[i].vertex1[0] - xAverage) / xRange;
		triangles[i].vertex1[1] = (triangles[i].vertex1[1] - yAverage) / yRange;
		triangles[i].vertex1[2] = (triangles[i].vertex1[2] - zAverage) / zRange;

		triangles[i].vertex2[0] = (triangles[i].vertex2[0] - xAverage) / xRange;
		triangles[i].vertex2[1] = (triangles[i].vertex2[1] - yAverage) / yRange;
		triangles[i].vertex2[2] = (triangles[i].vertex2[2] - zAverage) / zRange;

		triangles[i].vertex3[0] = (triangles[i].vertex3[0] - xAverage) / xRange;
		triangles[i].vertex3[1] = (triangles[i].vertex3[1] - yAverage) / yRange;
		triangles[i].vertex3[2] = (triangles[i].vertex3[2] - zAverage) / zRange;

	}
}