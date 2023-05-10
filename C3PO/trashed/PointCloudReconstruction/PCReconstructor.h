/* Reconstructor.cpp:
*	Contains functionality for reconstructing point cloud
*	partitions.
*  authors:
*	Froilan Luna-Lopez
*		University of Nevada, Reno
*  date: February 11, 2023
*/

#ifndef _PCReconstructor_H_
#define _PCReconstructor_H_

#include <string>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

bool DirExist(const std::string s);

void PCRotatorX(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, pcl::PointCloud<pcl::PointXYZ>::Ptr dest, float degrees);

void GetAcceptedFiles(std::vector<std::string>& dest, const std::string& dirName, const std::string& prefix);

bool MergePC(pcl::PointCloud<pcl::PointXYZ>::Ptr& dest,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc1,
	const unsigned int iters,
	const float dist,
	const float rot,
	const float trans);

pcl::PointCloud<pcl::PointXYZ>::Ptr DirMerge(const std::string& dirName, 
	const std::string& prefix,
	const unsigned int iters,
	const float dist,
	const float rot,
	const float trans,
	const unsigned int algo);

#endif