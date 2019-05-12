#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

class octree_change_detection
{
public:
	octree_change_detection()
	{

	}
	~octree_change_detection()
	{

	}
	
	void getSpatialChanges(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr partial_cloud, double resolutionInput);

};