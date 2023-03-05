#include <iostream>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "Voter.h"

int main()
{
	PointCloud bigPointCloud;
	int degree = 90;

	for (int i = 0; i < 4; i++) {
		PointCloud aPointCloud;
		std::string temp = "res/data" + std::to_string(i) + (std::string)".xyz";
		aPointCloud.readXYZFile(temp);
		aPointCloud.moveOrigin('z');
		aPointCloud.rotatePoints('y', i * 90);
		temp = "res/test" + std::to_string(i) + (std::string)".xyz";
		aPointCloud.writeXYZFile(temp);
	}

	bigPointCloud.readXYZFile("res/test0.xyz");
	bigPointCloud.readXYZFile("res/test1.xyz");
	bigPointCloud.readXYZFile("res/test2.xyz");
	bigPointCloud.readXYZFile("res/test3.xyz");

	bigPointCloud.writeXYZFile("res/megamind.xyz");

	return 0;
}
