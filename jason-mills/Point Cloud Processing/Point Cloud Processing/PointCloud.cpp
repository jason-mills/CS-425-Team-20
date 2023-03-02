#include "PointCloud.h"

std::vector<std::string> PointCloud::split(std::string line)
{
	std::stringstream ss(line);
	std::string word;
	std::vector<std::string> words;

	while (ss >> word)
	{
		words.push_back(word);
	}

	return words;
}

PointCloud::PointCloud() {}

PointCloud::PointCloud(std::vector<Eigen::Matrix<float, 4, 1>> newPoints) 
{
	points = newPoints;
}

PointCloud::PointCloud(const PointCloud& aPointCloud) 
{
	points = aPointCloud.points;
}

std::vector< Eigen::Matrix<float, 4, 1>> PointCloud::getPoints() 
{
	return points;
}

void PointCloud::addPoint(Eigen::Matrix<float, 4, 1> aPoint)
{
	points.push_back(aPoint);
}

void PointCloud::addPointCloud(std::vector<Eigen::Matrix<float, 4, 1>> aPointCloud)
{
	points = aPointCloud;
}

void PointCloud::rotatePoints(char axis, int degrees)
{
	Eigen::Matrix<float, 4, 4> rotator;
	
	rotationAngle = degrees;

	switch (axis)
	{
	case('x'):
		rotator <<
			1, 0, 0, 0,
			0, (cos(rotationAngle * radianConverter)), (sin(rotationAngle * radianConverter)), 0,
			0, -(sin(rotationAngle * radianConverter)), (cos(rotationAngle * radianConverter)), 0,
			0, 0, 0, 1;
		break;
	case('y'):
		rotator <<
			(cos(rotationAngle * radianConverter)), 0, -(sin(rotationAngle * radianConverter)), 0,
			0, 1, 0, 0,
			(sin(rotationAngle * radianConverter)), 0, (cos(rotationAngle * radianConverter)), 0,
			0, 0, 0, 1;
		break;
	case('z'):
		rotator <<
			(cos(rotationAngle * radianConverter)), (sin(rotationAngle * radianConverter)), 0, 0,
			-(sin(rotationAngle * radianConverter)), (cos(rotationAngle * radianConverter)), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;
	default:
		throw std::invalid_argument("Must use x, y, or z to define axis");
	}

	for (int i = 0; i < points.size(); i++) {
		points[i] = rotator * points[i];
	}

	return;
}

void PointCloud::print()
{
	for (int i = 0; i < points.size(); i++)
	{
		std::cout << "{\n" << points[i] << "\n}" << std::endl;
	}
}

void PointCloud::readPCDFile()
{
	//implement read here
}

void PointCloud::writePCDFile()
{
	//implement read here
}

void PointCloud::readXYZFile(std::string filePath)
{
	std::ifstream file;
	file.open(filePath);

	std::string line;
	int i = 0;

	getline(file, line);
	if (line == "X Y Z")
	{
		std::cout << line << std::endl;
	}

	while (getline(file, line))
	{
		std::vector<std::string> values = split(line);
		if (std::stof(values[0]) == 0 && std::stof(values[1]) == 0 && std::stof(values[2]) == 0)
		{
			continue;
		}
		//std::cout << values[0] << ' ' << values[1] << ' ' << values[2] << std::endl;
		addPoint({ std::stof(values[0]), std::stof(values[1]), std::stof(values[2]), 0 });
			
		if (i > 1)
			break;
		i++;
	}

	file.close();

	return;
}

void PointCloud::writeXYZFile(std::string filePath)
{
	std::ofstream file(filePath);

	file << "X Y Z";

	for (int i = 0; i < points.size(); i++) {
		file << "\n" << points[i][0]  << ' ' << points[i][1] << ' ' << points[i][2];
	}

	file.close();
}
