#ifndef _PCFILTER_FILTER_H_
#define _PCFILTER_FILTER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void PCRemoveOutliersXYZ(const std::vector<pcl::PointCloud<pcl::PointXYZ>> source,
	std::vector<pcl::PointCloud<pcl::PointXYZ>> dest);

#endif