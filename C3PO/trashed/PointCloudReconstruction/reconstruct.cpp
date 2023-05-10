/* reconstruct.cpp:
*	This program takes a directory with .ply files of the
*	form {prefix}{0...n}.ply. These .ply files are then
*	registered/combined to form one final .ply file.
*  authors:
*	Froilan Luna-Lopez
*		University of Nevada, Reno
*  date: February 14, 2023
*/

// Libraries
#include <boost/program_options.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "PCReconstructor.h"
#include "PCFormater.h"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace po = boost::program_options;

int main(int argc, char** argv) {
	boost::program_options::variables_map vm;

	try {
		// Supported options
		boost::program_options::options_description desc("Allowed options");
		desc.add_options()
			("help", "Display this help message")
			("dir", po::value<std::string>(), "Absolute path to directory containing files to register")
			("prefix", po::value<std::string>(), "Prefix of files to merge ('{prefix}{0...n}.ply')")
			("out", po::value<std::string>(), "Absolute path to output resulting file to")
			("iters", po::value<unsigned int>()->default_value(50), "The number of iterations to run the registration algorithm")
			("dist", po::value<float>()->default_value(1e-9), "The maximum distance to search for matching points in registration")
			("rot", po::value<float>()->default_value(1e-2), "The maximum rotation difference between icp iterations")
			("trans", po::value<float>()->default_value(1e-2), "The maximum translation difference between icp iterations")
			("algo", po::value<unsigned int>()->default_value(1), "The algorithm to use for registration (0=icp, 1=icp with normals)")
			;

		// Parse given command-line options
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
		boost::program_options::notify(vm);

		// Test for specific command-line options
		if (vm.count("help") != 0) {
			std::cout << desc << std::endl;
			return 1;
		}
		if (vm.count("dir") == 0) {
			std::cout << desc << std::endl;
			return 1;
		}
		if (vm.count("prefix") == 0) {
			std::cout << desc << std::endl;
			return 1;
		}
		if (vm.count("out") == 0) {
			std::cout << desc << std::endl;
			return 1;
		}
		if (vm.count("iters") > 1 || vm.count("iters") < 0) {
			std::cout << desc << std::endl;
			return 1;
		}
	}
	// Test for errors in command-line parsing
	catch (const po::error& e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}

	// Merge point clouds within directory
	pcl::PointCloud<pcl::PointXYZ>::Ptr merged = DirMerge(vm["dir"].as<std::string>(), 
		vm["prefix"].as<std::string>(),
		vm["iters"].as<unsigned int>(),
		vm["dist"].as<float>(),
		vm["rot"].as<float>(),
		vm["trans"].as<float>(),
		vm["algo"].as<unsigned int>());

	// Save to disk
	if (pcl::io::savePLYFileBinary(vm["out"].as<std::string>(), *merged) != 0) {
		std::cout << "Error: Could not save to file '" << vm["out"].as<std::string>() << "'" << std::endl;
		PCL_ERROR("Could not save file\n");
		return 1;
	}

	return 0;
}