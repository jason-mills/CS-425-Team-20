#pragma once
#define _USE_MATH_DEFINES

#include <vector>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>


class PointCloud
{
private: 
	int rotationAngle = 0;
	double radianConverter = M_PI / 180;
	float xAvg = 0, yAvg = 0, zAvg = 0;

	std::vector< Eigen::Matrix<float, 4, 1>> points;

	std::vector<std::string> split(std::string line);

	void calcAverages();

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

	void readXYZFile(std::string filePath);
	void writeXYZFile(std::string filePath);

	void print();
};

