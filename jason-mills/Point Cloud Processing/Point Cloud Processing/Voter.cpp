#include "Voter.h"

void Voter::calcAverages()
{
	float xSum = 0;
	float ySum = 0;
	float zSum = 0; 

	for (int i = 0; i < xPoints.size(); i++)
	{
		xSum += xPoints[i];
	}

	for (int i = 0; i < yPoints.size(); i++)
	{
		ySum += yPoints[i];
	}

	for (int i = 0; i < zPoints.size(); i++)
	{
		zSum += zPoints[i];
	}

	xAvg = xSum / xPoints.size();
	yAvg = ySum / yPoints.size();
	zAvg = zSum / zPoints.size();
}

void Voter::calcDeviations()
{
	float xVarianceSum = 0;
	float yVarianceSum = 0;
	float zVarianceSum = 0;

	for (int i = 0; i < xPoints.size(); i++)
	{
		xVarianceSum += pow(xAvg - xPoints[i], 2);
	}

	for (int i = 0; i < yPoints.size(); i++)
	{
		yVarianceSum += pow(yAvg - yPoints[i], 2);
	}

	for (int i = 0; i < zPoints.size(); i++)
	{
		zVarianceSum += pow(zAvg - zPoints[i], 2);
	}

	xDeviation = sqrt(xVarianceSum / xPoints.size());
	yDeviation = sqrt(yVarianceSum / yPoints.size());
	zDeviation = sqrt(zVarianceSum / zPoints.size());
}

float Voter::calcZScore(float value, float deviation, float average)
{
	return (value - average) / deviation;
}


Voter::Voter() :xAvg(0), yAvg(0), zAvg(0), xDeviation(0), yDeviation(0), zDeviation(0) {}

Voter::Voter(std::vector<float> newXPoints, std::vector<float> newYPoints, std::vector<float> newZPoints): xAvg(0), yAvg(0), zAvg(0), xDeviation(0), yDeviation(0), zDeviation(0), xPoints(newXPoints), yPoints(newYPoints) , zPoints(newZPoints) {}

void Voter::addPoints(std::vector<float> newXPoints, std::vector<float> newYPoints, std::vector<float> newZPoints)
{
	xPoints = newXPoints;
	yPoints = newYPoints;
	zPoints = newZPoints;
}

Eigen::Matrix<float, 4, 1> Voter::vote()
{
	calcAverages();
	calcDeviations();

	float xSum = 0;
	int xCount = 0;
	float ySum = 0;
	int yCount = 0;
	float zSum = 0;
	int zCount = 0;

	for (int i = 0; i < xPoints.size(); i++)
	{
		if (calcZScore(xPoints[i], xDeviation, xAvg) < 1)
		{
			xSum += xPoints[i];
			xCount++;
		}
	}

	for (int i = 0; i < yPoints.size(); i++)
	{
		if (calcZScore(yPoints[i], yDeviation, yAvg) < 1)
		{
			ySum += yPoints[i];
			yCount++;
		}
	}

	for (int i = 0; i < zPoints.size(); i++)
	{
		if (calcZScore(zPoints[i], zDeviation, zAvg) < 1)
		{
			zSum += zPoints[i];
			zCount++;
		}
	}

	return Eigen::Matrix<float, 4, 1> {(xSum / xCount), (ySum / yCount), (zSum / zCount), 0};
}
