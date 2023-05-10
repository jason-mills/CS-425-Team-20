/* Reconstructor.cpp:
*	Contains functionality for reconstructing point cloud
*	partitions.
*  authors:
*	Froilan Luna-Lopez
*		University of Nevada, Reno
*  date: February 12, 20233
*/


#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/make_shared.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <string.h>
#include <filesystem>
#include <vector>
#include <iostream>
#include <map>

#include "PCReconstructor.h"
#include "PCFormater.h"

namespace fs = std::filesystem;

/* DirExist():
*	Tests whether a given string corresponds to
*	an actual directory on the system.
*  args:
*	@s: Path to test.
*  return:
*	true: If given path exists.
*	false: Otherwise.
*/
bool DirExist(const std::string s) {
	if (!fs::exists(fs::path(s))) {
		return false;
	}

	return true;
}

/* PCRotatorX():
*	Rotates a point cloud about an axis by
*	a set of given degrees.
* args:
*	@pc: Point cloud to rotate.
*	@degrees: Degrees to rotate about the x-axis.
* return:
*	@pc
*/
void PCRotatorX(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr dest, float degrees) {
	// Create matrix for rotation about z-axis
	Eigen::Affine3f transZ = Eigen::Affine3f::Identity();
	transZ.rotate(Eigen::AngleAxisf(degrees, Eigen::Vector3f::UnitY()));

	// Perform rotation
	pcl::transformPointCloud(*pc, *dest, transZ);
}

/* PCSwitchSidesY():
*	Transitions a point cloud to the opposite side of the XZ hyperplane.
* args:
*	@pc: Point cloud to transition.
*	@dest: Point cloud to save transition state to.
* return:
*	@dest
*/
bool PCSwitchSidesY(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr dest) {
	Eigen::Matrix4f transXZ = Eigen::Matrix4f::Identity();
	transXZ(0, 0) = -1;
	//transXZ(2, 2) = -1;

	pcl::transformPointCloud(*pc, *dest, transXZ);

	return 0;
}

/* PCTransXZ():
*	Translate a point cloud across the XZ hyperplane.
* args:
*	@pc: Point cloud to transition
*	@dest: Point cloud to save transition state to
*	@dist: Distance to translate
* return:
*	@dest
*/
bool PCTransXZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr pc, 
	pcl::PointCloud<pcl::PointXYZ>::Ptr dest,
	float dist) {
	Eigen::Matrix4f transXZ = Eigen::Matrix4f::Identity();
	transXZ(0, 3) = dist;
	transXZ(2, 3) = dist;

	pcl::transformPointCloud(*pc, *dest, transXZ);

	return 0;
}

/* PCGetMinToOrigin():
*	Obtains the minimum distance from a point cloud to the origin.
* args:
*	@pc: Point cloud to iterate through.
* return:
*	Minimum distance from a point cloud to the origin
*/
float PCGetMinToOrigin(const pcl::PointCloud<pcl::PointXYZ> pc) {
	float min = LONG_MAX;
	for (size_t i = 0; i < pc.size(); i++) {
		// Extract data point
		pcl::PointXYZ point = pc[i];
		float x = point.x;
		float y = point.y;
		float z = point.z;

		// Calculate distance from origin
		float dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2));

		// Test for new minimum
		if (dist < min) min = dist;
	}

	return min;
}

int ExtractPathDegrees(std::string source) {
	// Extract prefix name
	std::string removeExt = source.substr(0, source.find_last_of("0123456789") + 1);
	std::string removePre = removeExt.substr(source.find_first_of("123456789"), source.size());
	return std::stoi(removePre);
}

/* GetAcceptedFiles():
*	Iterates through a given given directory and collects all valid
*	point cloud files. Files should match the format of
*	{prefix}{0...n}.ply to be considered a valid file.
*  args:
*	@dest: Where to store the accepted files and their paths.
*	@dirName: The path to the directory that will be iterated through.
*	@prefix: The prefix to match when iterating through the files.
*  return:
*	@dest
*/
void GetAcceptedFiles(std::vector<std::string>& dest, const std::string& dirName, const std::string& prefix) {
	// Variables
	std::map<int, std::string> orderedFiles;

	// Go through all files in given directory
	for (auto const& dir_entry : fs::directory_iterator(dirName)) {
		// Ignore directories
		if (!dir_entry.is_regular_file()) {
			continue;
		}

		// Extract file name
		std::string filePath = dir_entry.path().string();
		size_t fileCutPos = filePath.find_last_of("/\\") + 1;

		// Extract extension
		std::string fileName = filePath.substr(fileCutPos, filePath.length());
		size_t extensionPos = fileName.rfind(".");

		// Ignore file if extension is not ".ply"
		if (extensionPos == -1 || fileName.substr(extensionPos, filePath.length()).compare(".ply") != 0) {
			continue;
		}

		// Extract prefix name
		std::string filePrefix = fileName.substr(0, fileName.find_first_of("0123456789"));
		if (filePrefix.compare(prefix) == 0) {
			// Get file number
			int fileNumStart = fileName.find_first_of("0123456789");
			int fileNumLen = fileName.find_last_of("0123456789") - fileNumStart;
			std::string fileNum = fileName.substr(fileNumStart, fileNumLen + 1);
			int num = std::stoi(fileNum);
			orderedFiles[num] = filePath;
		}
	}

	// Push ordered results to output destination
	for (auto const& item : orderedFiles) {
		dest.push_back(item.second);
	}
}

void cleanPC(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest) {
	// Remove NaN points
	std::vector<int> indices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr noNanCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::removeNaNFromPointCloud(*source, *dest, indices);

	//std::cout << dest << std::endl;
}

int fileNum = 0;

void calcPointNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointNormal>::Ptr dest) {
	// Initialize variables
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceClean(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	// Clean source
	cleanPC(source, sourceClean);

	// Match sizes
	normals->resize(source->size());

	// Initialize parameters for estimating normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(sourceClean);
	ne.setRadiusSearch(0.03);
	ne.setSearchMethod(tree);

	// Compute normals
	ne.compute(*normals);

	// Combine points with normals
	pcl::concatenateFields(*sourceClean, *normals, *dest);
}

/* MergePCNormal():
*	Merges two point clouds together using iterative closest point
*	and saves the resulting point cloud into another point cloud.
*  args:
*	@dest: Point cloud to save the resulting merged point cloud
*		into.
*	@pc1: One of the point clouds to be merged with another.
*	@pc2: Another point cloud to merge with @pc1.
*  return:
*	true: If the given point clouds merged successfully.
*	false: Otherwise.
*/
bool MergePCNormal(pcl::PointCloud<pcl::PointNormal>::Ptr& pc1,
	const pcl::PointCloud<pcl::PointNormal>::Ptr& pc2) {
	// Configure ply files
	pc1->is_dense = false;
	pc2->is_dense = false;

	// Filter point clouds
	std::vector<int> indices;
	pcl::PointCloud<pcl::PointNormal>::Ptr plyFilter1(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr plyFilter2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::removeNaNFromPointCloud(*pc1, *plyFilter1, indices);
	pcl::removeNaNFromPointCloud(*pc2, *plyFilter2, indices);

	// Register ply files
	std::cout << "Aligning point clouds..." << std::endl;
	pcl::PointCloud<pcl::PointXYZ> Final;
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(plyFilter1);
	icp.setInputTarget(plyFilter2);
	icp.setTransformationEpsilon(1e-9);
	icp.setEuclideanFitnessEpsilon(1e-3);
	icp.setMaxCorrespondenceDistance(0.2);
	//icp.setRANSACOutlierRejectionThreshold(1.5);
	icp.setMaximumIterations(50);
	icp.align(*plyFilter1);

	if (icp.hasConverged()) {
		std::cout << "ICP has converged" << std::endl;
		*plyFilter1 += *plyFilter2;
	}
	else {
		std::cout << "ICP has not converged" << std::endl;
	}
	
	//// Save to disk
	//std::string tmpSave = "C:\\Users\\froil\\Downloads\\bunny\\data\\" + std::to_string(fileNum) + ".ply";
	//if (pcl::io::savePLYFileBinary(tmpSave, *plyFilter1) != 0) {
	//	std::cout << "Error: Could not save to file '" << tmpSave << "'" << std::endl;
	//	PCL_ERROR("Could not save file\n");
	//	return 1;
	//}
	fileNum++;

	// Extract point cloud pointer
	//dest = pcl::PointCloud<pcl::PointXYZ>::Ptr(Final.makeShared());
	//dest = Final.makeShared();
	pc1 = plyFilter1;

	return true;
}

/* MergePC():
*	Merges two point clouds together using iterative closest point
*	and saves the resulting point cloud into another point cloud.
*  args:
*	@dest: Point cloud to save the resulting merged point cloud
*		into.
*	@pc1: One of the point clouds to be merged with another.
*	@pc2: Another point cloud to merge with @pc1.
*  return:
*	true: If the given point clouds merged successfully.
*	false: Otherwise.
*/
bool MergePC(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc1,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc2,
	const unsigned int iters,
	const float dist,
	const float rot,
	const float trans) {
	// Configure ply files
	pc1->is_dense = false;
	pc2->is_dense = false;

	// Register ply files
	std::cout << "Aligning point clouds..." << std::endl;
	pcl::PointCloud<pcl::PointXYZ> Final;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(pc2);
	icp.setInputTarget(pc1);
	icp.setTransformationEpsilon(trans);
	icp.setTransformationRotationEpsilon(rot);
	icp.setEuclideanFitnessEpsilon(1e-24);
	icp.setMaxCorrespondenceDistance(dist);
	//icp.setRANSACOutlierRejectionThreshold(1.5);
	icp.setMaximumIterations(iters);
	icp.align(Final);

	if (icp.hasConverged()) {
		std::cout << "ICP has converged" << std::endl;
		Final += *pc1;
	}
	else {
		std::cout << "ICP has not converged" << std::endl;
	}

	// Save to disk
	/*std::string tmpSave = "C:\\Users\\froil\\Downloads\\bunny\\data\\" + std::to_string(fileNum) + ".ply";
	if (pcl::io::savePLYFileASCII(tmpSave, *pc1) != 0) {
		std::cout << "Error: Could not save to file '" << tmpSave << "'" << std::endl;
		PCL_ERROR("Could not save file\n");
		return 1;
	}
	fileNum++;*/
	*pc1 = Final;

	return true;
}

/* DirMergeNormal():
*	Merges all point cloud files within a directory that match the
*	file format {prefix}{0...n}.ply. All other files are ignored.
*  args:
*	@dirName: The path to the directory that will be iterated through.
*	@prefix: The prefix to match when iterating through the files.
*  return:
*	A point cloud that is the result from merging all the accepted
*	point clouds within the given directory.
*	Will return nullptr if the given directory does not exist.
*	Will return nullptr if there are <= 1 valid files.
*	Will return nullptr if any valid file could not be opened.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr DirMergeNormal(const std::string& dirName, const std::string& prefix) {
	// Test if directory exists, otherwise return false
	if (!DirExist(dirName)) {
		std::cerr << "Error: Directory '" << dirName << "' could not be found" << std::endl;
		return nullptr;
	}

	// Extract all files that match {prefix}{INT}.ply
	std::vector<std::string> fileParts = std::vector<std::string>();
	GetAcceptedFiles(fileParts, dirName, prefix);
	
	// Showcase accepted files
	std::cout << "Accepted files:" << std::endl;
	for (auto const& s : fileParts) {
		std::cout << s << std::endl;
	}
	std::cout << std::endl;

	// Test for sufficient files to merge
	if (fileParts.size() <= 1) {
		std::cerr << "Error: Insufficient accepted files" << std::endl;
		return nullptr;
	}

	// Variables to track point cloud iterations
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);

	// Load in the initial base point cloud
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileParts[0], *source) == -1) {
		std::cout << "Couldn't read file '" << fileParts[0] << "'\n";
		PCL_ERROR("Could not open file\n");
		return nullptr;
	}

	// Configure initial point cloud
	source->is_dense = false;

	// Update file queue
	std::cout << "Base file: " << fileParts[0] << std::endl;
	fileParts.erase(fileParts.begin());

	// Calculate normals for point cloud
	std::cout << "Calculating initial normals..." << std::endl;
	pcl::PointCloud<pcl::PointNormal>::Ptr sourceNormal(new pcl::PointCloud<pcl::PointNormal>);
	calcPointNormals(source, sourceNormal);

	// Iteratively merge encountered point clouds
	int mergeIter = 1;
	for (auto const& filePath : fileParts) {
		if (pcl::io::loadPLYFile<pcl::PointXYZ>(filePath, *target) == -1) {
			std::cout << "Couldn't read file '" << filePath << "'\n";
			PCL_ERROR("Could not open file\n");
			return nullptr;
		}
		target->is_dense = false;

		std::cout << "Merging with " << filePath << "..." << std::endl;

		std::cout << "Calculating normals..." << std::endl;
		pcl::PointCloud<pcl::PointNormal>::Ptr targetNormal(new pcl::PointCloud<pcl::PointNormal>);
		calcPointNormals(target, targetNormal);

		// Merge the next point cloud in-line with the current source
		MergePCNormal(sourceNormal, targetNormal);
	}
	std::cout << std::endl;

	std::cout << "Successfully completed merge" << std::endl;

	//pcl::visualization::CloudViewer viewer("Simple cloud viewer");
	//viewer.showCloud(merged);
	return source;
}

/* DirMerge():
*	Merges all point cloud files within a directory that match the
*	file format {prefix}{0...n}.ply. All other files are ignored.
*  args:
*	@dirName: The path to the directory that will be iterated through.
*	@prefix: The prefix to match when iterating through the files.
*  return:
*	A point cloud that is the result from merging all the accepted
*	point clouds within the given directory.
*	Will return nullptr if the given directory does not exist.
*	Will return nullptr if there are <= 1 valid files.
*	Will return nullptr if any valid file could not be opened.
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr DirMerge(const std::string& dirName, 
	const std::string& prefix,
	const unsigned int iters,
	const float dist,
	const float rot,
	const float trans,
	const unsigned int algo) {
	// Test if directory exists, otherwise return false
	if (!DirExist(dirName)) {
		std::cerr << "Error: Directory '" << dirName << "' could not be found" << std::endl;
		return nullptr;
	}

	// Extract all files that match {prefix}{INT}.ply
	std::vector<std::string> fileParts = std::vector<std::string>();
	GetAcceptedFiles(fileParts, dirName, prefix);

	// Showcase accepted files
	std::cout << "Accepted files:" << std::endl;
	for (auto const& s : fileParts) {
		std::cout << s << std::endl;
	}
	std::cout << std::endl;

	// Test for sufficient files to merge
	if (fileParts.size() <= 1) {
		std::cerr << "Error: Insufficient accepted files" << std::endl;
		return nullptr;
	}

	// Variables to track point cloud iterations
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);

	// Load in the initial base point cloud
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileParts[0], *source) == -1) {
		std::cout << "Couldn't read file '" << fileParts[0] << "'\n";
		PCL_ERROR("Could not open file\n");
		return nullptr;
	}

	// Configure initial point cloud
	source->is_dense = false;

	// Created filtered point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceClean(new pcl::PointCloud<pcl::PointXYZ>);
	cleanPC(source, sourceClean);

	// Update file queue
	std::cout << "Base file: " << fileParts[0] << std::endl;
	fileParts.erase(fileParts.begin());

	// Iteratively merge encountered point clouds
	int mergeIter = 1;
	for (auto const& filePath : fileParts) {
		if (pcl::io::loadPLYFile<pcl::PointXYZ>(filePath, *target) == -1) {
			std::cout << "Couldn't read file '" << filePath << "'\n";
			PCL_ERROR("Could not open file\n");
			return nullptr;
		}
		target->is_dense = false;

		// Filter point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr targetClean(new pcl::PointCloud<pcl::PointXYZ>);
		cleanPC(target, targetClean);

		std::cout << "Merging with " << filePath << "..." << std::endl;

		// Merge the next point cloud in-line with the current source
		MergePC(sourceClean, targetClean, iters, dist, rot, trans);
	}
	std::cout << std::endl;

	std::cout << "Successfully completed merge" << std::endl;

	
	return sourceClean;
}