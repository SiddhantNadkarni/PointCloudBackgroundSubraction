#include "octree_change_detection.hpp"

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::string s)
{

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (s));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  // viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void octree_change_detection::getSpatialChanges(pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr partial_cloud, double resolutionInput)
{
	srand ((unsigned int) time (NULL));

	// Octree resolution - side length of octree voxels
	double resolution = resolutionInput;

	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
	

	// Instantiate octree-based point cloud change detection class
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

	  // Add points from cloudA to octree
	octree.setInputCloud (full_cloud);
	octree.addPointsFromInputCloud ();

	octree.switchBuffers();
	octree.setInputCloud(partial_cloud);
	octree.addPointsFromInputCloud();
	std::vector<int> changedIndices;

	pcl::PointIndices::Ptr inliers (new::pcl::PointIndices());
	octree.getPointIndicesFromNewVoxels(inliers->indices);
	newCloud->points.resize(480*640);

	// for (int i = 0; i < changedIndices.size(); ++i)
	// {
	// 	newCloud->points[changedIndices[i]].x = full_cloud->points[changedIndices[i]].x;
	// 	newCloud->points[changedIndices[i]].y = full_cloud->points[changedIndices[i]].y;
	// 	newCloud->points[changedIndices[i]].z = full_cloud->points[changedIndices[i]].z;
	// }
	pcl::ExtractIndices<pcl::PointXYZ> filter;

	filter.setInputCloud(full_cloud);
	filter.setIndices(inliers);
	filter.setNegative(false);
	filter.filter(*newCloud);

	pcl::visualization::PCLVisualizer::Ptr viewer;
	viewer = simpleVis(newCloud, "Plane_Segmented_Output_0");


    while (!viewer->wasStopped ())
    {
    	viewer->spinOnce (100);
    }

}




