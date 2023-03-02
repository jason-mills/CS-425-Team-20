#include <iostream>
#include <Eigen/Dense>
#include "PointCloud.h"
#include "Voter.h"

int main()
{
	PointCloud aPointCloud;

	//aPointCloud.readXYZFile("res/data.xyz");

	aPointCloud.addPoint({ 1, 0, 0, 0 });
	aPointCloud.addPoint({ 1, 0, 0, 0 });
	aPointCloud.addPoint({ 1, 0, 0, 0 });

	aPointCloud.rotatePoints('z', -90);

	aPointCloud.writeXYZFile("res/test.xyz");

	return 0;
}
