#include <iostream>
#include <Eigen/Dense>
#include "PointCloud.h"

int main()
{
	PointCloud aPointCloud;

	Eigen::Matrix<float, 4, 1> point{ 1, 0, 0, 0 };

	aPointCloud.addPoint(point);

	aPointCloud.rotatePoints('y', 90);


	std::vector<Eigen::Matrix<float, 4, 1>> points = aPointCloud.getPoints();

	for (int i = 0; i < points.size(); i++) 
	{
		std::cout << (points[i]) << std::endl;
	}
	

	return 0;
}
