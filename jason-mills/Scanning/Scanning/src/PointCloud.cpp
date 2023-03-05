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

void PointCloud::calcAverages()
{
	float xSum = 0;
	float ySum = 0;
	float zSum = 0;

	for (int i = 0; i < points.size(); i++)
	{
		xSum += points[i][0];
		ySum += points[i][1];
		zSum += points[i][2];
	}

	xAvg = xSum / points.size();
	yAvg = ySum / points.size();
	zAvg = zSum / points.size();

	return;
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

	return;
}

void PointCloud::addPointCloud(std::vector<Eigen::Matrix<float, 4, 1>> aPointCloud)
{
	points.insert(points.end(), aPointCloud.begin(), aPointCloud.end());

	return;
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

	return;
}

void PointCloud::readPCDFile()
{
	//implement read here

	return;
}

void PointCloud::writePCDFile()
{
	//implement read here

	return;
}

void PointCloud::readXYZFile(std::string filePath)
{
	std::ifstream file;
	file.open(filePath);

	std::string line;
	int i = 0;

	getline(file, line);
	if (line != "X Y Z")
	{
		throw std::invalid_argument("File is not of type X Y Z");
	}

	while (getline(file, line))
	{
		std::vector<std::string> values = split(line);
		if (std::stof(values[0]) == 0 && std::stof(values[1]) == 0 && std::stof(values[2]) == 0)
		{
			continue;
		}

		addPoint({ std::stof(values[0]), std::stof(values[1]), std::stof(values[2]), 0 });
	}

	file.close();

	return;
}

void PointCloud::writeXYZFile(std::string filePath)
{
	std::ofstream file(filePath);

	file << "X Y Z";

	for (int i = 0; i < points.size(); i++) {
		file << "\n" << points[i][0] << ' ' << points[i][1] << ' ' << points[i][2];
	}

	file.close();

	return;
}

void PointCloud::moveOrigin(char axis, float moveFactor)
{
	float amountToMoveBy = 0.14;

	int indexToAlter = -1;
	switch (axis)
	{
	case('x'):
		indexToAlter = 0;
		break;
	case('y'):
		indexToAlter = 1;
		break;
	case('z'):
		indexToAlter = 2;
		break;
	default:
		throw std::invalid_argument("Must use x, y, or z to define axis");
	}

	std::cout << "Amount to move by: " << amountToMoveBy << std::endl;

	for (int i = 0; i < points.size(); i++)
	{
		points[i][indexToAlter] -= amountToMoveBy;
	}

	return;
}
