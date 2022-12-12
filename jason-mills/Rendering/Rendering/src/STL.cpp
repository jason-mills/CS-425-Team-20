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
		if (temp.vertex1[1] < xMin) xMin = temp.vertex1[1];
		if (temp.vertex1[2] < xMin) xMin = temp.vertex1[2];
		if (temp.vertex1[0] > xMax) xMax = temp.vertex1[0];
		if (temp.vertex1[1] > xMax) xMax = temp.vertex1[1];
		if (temp.vertex1[2] > xMax) xMax = temp.vertex1[2];

		if (temp.vertex2[0] < yMin) yMin = temp.vertex2[0];
		if (temp.vertex2[1] < yMin) yMin = temp.vertex2[1];
		if (temp.vertex2[2] < yMin) yMin = temp.vertex2[2];
		if (temp.vertex2[0] > yMax) yMax = temp.vertex2[0];
		if (temp.vertex2[1] > yMax) yMax = temp.vertex2[1];
		if (temp.vertex2[2] > yMax) yMax = temp.vertex2[2];

		if (temp.vertex3[0] < zMin) zMin = temp.vertex3[0];
		if (temp.vertex3[1] < zMin) zMin = temp.vertex3[1];
		if (temp.vertex3[2] < zMin) zMin = temp.vertex3[2];
		if (temp.vertex3[0] > zMax) zMax = temp.vertex3[0];
		if (temp.vertex3[1] > zMax) zMax = temp.vertex3[1];
		if (temp.vertex3[2] > zMax) zMax = temp.vertex3[2];
		triangles.push_back(temp);
	}

	file.close();

	return;
}