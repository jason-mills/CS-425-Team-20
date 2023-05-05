#include <iostream>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "Voter.h"

int main(int argc, char *argv[])
{
	if (argc != 6)
	{
		std::cout << "Incorrect number of arguments.\n" << "Example Usage: ./processXyzFiles.exe translationFactor degreesToRotate numberOfFiles baseFilename newFilename.xyz" << std::endl;
		return 1;
	}

	PointCloud bigPointCloud;
	float translationFactor = atof(argv[1]);
	int degreesToRotate = std::stoi(argv[2]);
	int numberOfFiles = std::stoi(argv[3]);
	std::string baseFilename = argv[4];
	std::string newFilename = argv[5];

	for (int i = 0; i < numberOfFiles; i++) {
		PointCloud aPointCloud;

		aPointCloud.readXYZFile(baseFilename + std::to_string(i) + (std::string)".xyz");

		aPointCloud.moveOrigin('z', translationFactor);
		aPointCloud.rotatePoints('y', i * degreesToRotate);

		bigPointCloud.addPointCloud(aPointCloud.getPoints());
	}

	bigPointCloud.writeXYZFile(newFilename);

	return 0;
}
