#include "pch.h"
#include <gtest/gtest.h>
#include "../Point Cloud Processing/PointCloud.h"
#include "../Point Cloud Processing/PointCloud.cpp"
#include <Eigen/Dense>

// C++ trig is very sad because the cos 90 is not 0 
// This means that we have to have tolerance when calculating the rotation of points
const double TOLERANCE = 0.0000001;

TEST(MatrixMultiplicationTest, TranslateX90Degrees) 
{
	PointCloud aPointCloud;
	aPointCloud.addPoint(Eigen::Matrix<float, 4, 1>{0, 1, 0, 0});
	aPointCloud.rotatePoints('x', 90);
	std::vector<Eigen::Matrix<float, 4, 1>> points = aPointCloud.getPoints();
	
	EXPECT_TRUE(points[0][0] == 0);
	EXPECT_TRUE(points[0][1] < 0 + TOLERANCE);
	EXPECT_TRUE(points[0][2] == -1);
}

TEST(MatrixMultiplicationTest, TranslateY90Degrees)
{
	PointCloud aPointCloud;
	aPointCloud.addPoint(Eigen::Matrix<float, 4, 1>{1, 0, 0, 0});
	aPointCloud.rotatePoints('y', 90);
	std::vector<Eigen::Matrix<float, 4, 1>> points = aPointCloud.getPoints();

	EXPECT_TRUE(points[0][0] < TOLERANCE);
	EXPECT_TRUE(points[0][1] == 0);
	EXPECT_TRUE(points[0][2] == 1);
}

TEST(MatrixMultiplicationTest, TranslateZ90Degrees)
{
	PointCloud aPointCloud;
	aPointCloud.addPoint(Eigen::Matrix<float, 4, 1>{1, 0, 0, 0});
	aPointCloud.rotatePoints('z', 90);
	std::vector<Eigen::Matrix<float, 4, 1>> points = aPointCloud.getPoints();

	EXPECT_TRUE(points[0][0] < 0 + TOLERANCE);
	EXPECT_TRUE(points[0][1] == -1);
	EXPECT_TRUE(points[0][2] == 0);
}

int main(int argc, char* argv[]) 
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}