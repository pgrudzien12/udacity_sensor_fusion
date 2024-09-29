/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
	for (int i = 0; i<maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		
		// Select two random indexes
		size_t l = cloud->points.size();
		int index1 = rand() % l;
		int index2, index3;
		do {
			index2 = rand() % l;
		} while (index1 == index2);
		do {
			index3 = rand() % l;
		} while (index3 == index2 || index3 == index1);


		// Get the points corresponding to the random indexes
		pcl::PointXYZ p1 = cloud->points[index1];
		pcl::PointXYZ p2 = cloud->points[index2];
		pcl::PointXYZ p3 = cloud->points[index3];
		inliers.insert(index1);
		inliers.insert(index2);
		inliers.insert(index3);
		// Ensure the points are not collinear by checking the area of the triangle they form
        Eigen::Vector3f v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        Eigen::Vector3f v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
        Eigen::Vector3f normal = v1.cross(v2);
        normal.normalize();  // Normalize the normal vector

        if (normal.norm() == 0) {
            continue; // Skip this iteration if the points are collinear
        }

        double A = normal[0];
        double B = normal[1];
        double C = normal[2];
		double D = -(A*p1.x + B*p1.y + C*p1.z);
		
		// Iterate through all points in the cloud
		for (int i = 0; i < cloud->points.size(); ++i) {
			pcl::PointXYZ point = cloud->points[i];
			
			// Calculate the perpendicular distance from the point to the line
			double distance = std::abs(A * point.x + B * point.y + C * point.x + D) / std::sqrt(A * A + B * B + C * C);

			// If the distance is within the tolerance, add the index to inliers
			if (distance <= distanceTol) {
				inliers.insert(i);
			}
		}

		if (inliersResult.size() < inliers.size())
			inliersResult = inliers;
	}    
	std::cout << "Inliers within distance tolerance:" << inliersResult.size() << std::endl;

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 1500, 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
