#include "STL.h"
#include <fstream>
#include <iostream>

STL::STL(std::string filepath) 
	: filepath(filepath)
{
	readFile();
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
		file.read(reinterpret_cast<char*>(&temp), sizeof(stlTriangle));
		std::cout << temp.vertex1[0] << " " << temp.vertex1[1] << " " << temp.vertex1[2] << " " << temp.color << std::endl;
		triangles.push_back(temp);
	}

	file.close();

	return;
}