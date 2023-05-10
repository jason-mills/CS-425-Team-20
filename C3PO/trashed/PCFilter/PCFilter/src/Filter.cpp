#include <boost/math/statistics/univariate_statistics.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <vector>

/* PCRemoveOutliersXYZ():
*	Removes points in sets of related point clouds that are
*	more than 1 z-scores away from the mean. Standard
*	deviation is determined by the median absolute deviation.
*	Outliers will be replaced by the mean values.
* args:
*	@source: Set containing point clouds to iterate through
*	@dest: Set to contain filtered point clouds
* return:
*	@dest
*/
void PCRemoveOutliersXYZ(const std::vector<pcl::PointCloud<pcl::PointXYZ>> source,
	std::vector<pcl::PointCloud<pcl::PointXYZ>> dest) {
	float zScoreMax = 1.0;

	// Initialize output point clouds as copies of input
	for (int i = 0; i < source.size(); i++) {
		pcl::copyPointCloud(source[i], dest[i]);
	}

	// Iterate through points
	for (int i = 0; i < (source.at(0)).size(); i++) {
		std::vector<float> xVals = std::vector<float>();
		std::vector<float> yVals = std::vector<float>();
		std::vector<float> zVals = std::vector<float>();
		float xMu, yMu, zMu;
		float xSDev, ySDev, zSDev;

		// Iterate through point clouds
		for (pcl::PointCloud<pcl::PointXYZ> pc : source) {
			// Extract XYZ values into XYZ vectors
			pcl::PointXYZ point = pc[i];
			xVals.push_back(point.x);
			yVals.push_back(point.y);
			zVals.push_back(point.z);
		}

		// Calculate the mean of XYZ vectors
		xMu = boost::math::statistics::mean(xVals);
		yMu = boost::math::statistics::mean(yVals);
		zMu = boost::math::statistics::mean(zVals);

		// Calculate the median absolute deviation of XYZ vectors
		xSDev = boost::math::statistics::median_absolute_deviation(xVals, xMu);
		ySDev = boost::math::statistics::median_absolute_deviation(yVals, yMu);
		zSDev = boost::math::statistics::median_absolute_deviation(zVals, zMu);

		// Test if any X vectors deviate too far from the mean
		for (int xi = 0; xi < xVals.size(); xi++) {
			if (std::abs(xi - xMu) / xSDev > zScoreMax) { // Test if z-score is greater than 3
				// Replace X value with X mean
				xVals.at(xi) = xMu;
			}
		}
		// Test if any Y vectors deviate too far from the mean
		for (int yi = 0; yi < yVals.size(); yi++) {
			if (std::abs(yi - yMu) / ySDev > zScoreMax) { // Test if z-score is greater than 3
				// Replace Y value with Y mean
				yVals.at(yi) = yMu;
			}
		}
		// Test if any Z vectors deviate too far from the mean
		for (int zi = 0; zi < zVals.size(); zi++) {
			if (std::abs(zi - zMu) / zSDev > zScoreMax) { // Test if z-score is greater than 3
				// Replace Z value with Z mean
				yVals.at(zi) = zMu;
			}
		}

		// Save modifications to output point clouds
		for (int j = 0; j < dest.size(); j++) {
			// Create point to add
			pcl::PointXYZ point;
			point.x = xVals[j];
			point.y = yVals[j];
			point.z = zVals[j];

			// Save point
			dest[j][i] = point;
		}
	}
}