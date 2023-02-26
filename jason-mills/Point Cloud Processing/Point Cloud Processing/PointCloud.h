#pragma once
#define _USE_MATH_DEFINES

#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>


class PointCloud
{
private: 
	int rotationAngle = 0;
	double radianConverter = M_PI / 180;

	std::vector< Eigen::Matrix<float, 4, 1>> points;

public:
	PointCloud();
	PointCloud(std::vector<Eigen::Matrix<float, 4, 1>> newPoints);
	PointCloud(const PointCloud &aPointCloud);
	
	std::vector< Eigen::Matrix<float, 4, 1>> getPoints();

	void addPoint(Eigen::Matrix<float, 4, 1> aPoint);
	void addPointCloud(std::vector<Eigen::Matrix<float, 4, 1>> aPointCloud);

	void rotatePoints(char axis, int degrees);

	void readPCDFile();
	void writePCDFile();

	void print();
};

