#include <iostream>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "Voter.h"

int main()
{
	PointCloud aPointCloud;

	aPointCloud.readXYZFile("res/data.xyz");

	aPointCloud.rotatePoints('x', -180);

	aPointCloud.writeXYZFile("res/dataXRotation.xyz");

	return 0;
}
