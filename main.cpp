#include "octree_change_detection.hpp"

int main(int argc, char const *argv[])
{
	octree_change_detection ocd;
	pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr partial_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// extracted_cloud->points.resize(480*640);
	pcl::PCDReader reader;
    reader.read (argv[1], *full_cloud);
    reader.read (argv[2], *partial_cloud);
    double resolutionInput = atof(argv[3]);
    ocd.getSpatialChanges(full_cloud, partial_cloud, resolutionInput);
    // extracted_cloud = getInliers(full_cloud, partial_cloud);
	return 0;
}