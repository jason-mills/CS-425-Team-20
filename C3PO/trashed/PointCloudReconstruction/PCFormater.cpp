#include <string>
#include <pcl/io/ply_io.h>

/* PlyAsciiToBinary():
*	Converts a .ply file with binary format into
*	a .ply file with ascii format
* args:
*	@filePath: Path to the file to be converted
*	@savePath: Path to the destination to save
*		the resulting file to
* return:
*	true: If successfully converted and saved
*		the resulting file.
*	false: Otherwise.
*/
bool PlyAsciiToBinary(std::string filePath, std::string savePath) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	// Load in the initial base point cloud
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filePath, *source) == -1) {
		std::cout << "Couldn't read file '" << filePath << "'\n";
		PCL_ERROR("Could not open file\n");
		return false;
	}

	// Save ascii as binary
	if (pcl::io::savePLYFileBinary(savePath, *source) != 0) {
		std::cout << "Error: Could not save to file '" << savePath << "'" << std::endl;
		PCL_ERROR("Could not save file\n");
		return false;
	}

	return true;
}

/* PlyBinaryToAscii():
*	Converts a .ply file with ascii format into
*	a .ply file with binary format
* args:
*	@filePath: Path to the file to be converted
*	@savePath: Path to the destination to save
*		the resulting file to
* return:
*	true: If successfully converted and saved
*		the resulting file.
*	false: Otherwise.
*/
bool PlyBinaryToAscii(std::string filePath, std::string savePath) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
	// Load in the initial base point cloud
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filePath, *source) == -1) {
		std::cout << "Couldn't read file '" << filePath << "'\n";
		PCL_ERROR("Could not open file\n");
		return false;
	}

	// Save ascii as binary
	if (pcl::io::savePLYFile(savePath, *source) != 0) {
		std::cout << "Error: Could not save to file '" << savePath << "'" << std::endl;
		PCL_ERROR("Could not save file\n");
		return false;
	}

	return true;
}