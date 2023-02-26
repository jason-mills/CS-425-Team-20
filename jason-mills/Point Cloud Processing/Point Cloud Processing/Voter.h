#pragma once
#include <Eigen/Dense>
#include <vector>
#include <math.h>

class Voter
{
private:
	std::vector<float> xPoints, yPoints, zPoints;
	
	float xAvg, yAvg, zAvg;
	float xDeviation, yDeviation, zDeviation;

	void calcAverages();
	void calcDeviations();
	float calcZScore(float value, float deviation, float average);

public:
	Voter();
	Voter(std::vector<float> newXPoints, std::vector<float> newYPoints, std::vector<float> newZPoints);

	void addPoints(std::vector<float> newXPoints, std::vector<float> newYPoints, std::vector<float> newZPoints);

	Eigen::Matrix<float, 4, 1> vote();
};