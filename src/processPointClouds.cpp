// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


template<typename PointT>
void Ransac(pcl::PointIndices &indices, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	srand(time(NULL));
	std::unordered_set<int> inliersResult;
	
	// For max iterations
	for (int i = 0; i<maxIterations; i++)
	{
		std::unordered_set<int> inliers;
		
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
		PointT* p1 = &cloud->points[index1];
		PointT* p2 = &cloud->points[index2];
		PointT* p3 = &cloud->points[index3];
		inliers.insert(index1);
		inliers.insert(index2);
		inliers.insert(index3);
		// Ensure the points are not collinear by checking the area of the triangle they form
        Eigen::Vector3f v1(p2->x - p1->x, p2->y - p1->y, p2->z - p1->z);
        Eigen::Vector3f v2(p3->x - p1->x, p3->y - p1->y, p3->z - p1->z);
        Eigen::Vector3f normal = v1.cross(v2);
        normal.normalize();  // Normalize the normal vector

        if (normal.norm() < 1e-6) {
            continue; // Skip this iteration if the points are almost collinear
        }

        double A = normal[0];
        double B = normal[1];
        double C = normal[2];
		double D = -(A*p1->x + B*p1->y + C*p1->z);
		
		// Iterate through all points in the cloud
		for (int j = 0; j < cloud->points.size(); ++j) {
			PointT* point = &cloud->points[j];
			
			// Calculate the perpendicular distance from the point to the plane
			double distance = std::abs(A * point->x + B * point->y + C * point->z + D) / std::sqrt(A * A + B * B + C * C);

			// If the distance is within the tolerance, add the index to inliers
			if (distance <= distanceTol) {
				inliers.insert(j);
			}
		}

		if (inliersResult.size() < inliers.size())
        {
	        //std::cout << "Got potential surface with "<< inliers.size()<< " of points and normal:" << normal << std::endl;
			inliersResult = inliers;
        }
	}    
	std::cout << "Inliers within distance tolerance:" << inliersResult.size() << std::endl;

	indices.indices.clear();
	indices.indices.insert(indices.indices.begin(), inliersResult.begin(), inliersResult.end()); // overload (4)
}

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered3 (new pcl::PointCloud<PointT>());
    //typename pcl::StatisticalMultiscaleInterestRegionExtraction<PointT> extractor;
    typename pcl::CropBox<PointT> cropBox;
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);

    cropBox.setInputCloud(cloud);
    cropBox.filter(*cloud_filtered);

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud_filtered);  
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered2);

    // Define the min and max bounds of the box
    Eigen::Vector4f min_point_car(-1.5f,-1.4,-2, 1.0); // x_min, y_min, z_min
    Eigen::Vector4f max_point_car(2.5,1.4,0.2, 1.0);   // x_max, y_max, z_max

    cropBox.setMin(min_point_car);
    cropBox.setMax(max_point_car);
    cropBox.setNegative(true);
    cropBox.setInputCloud(cloud_filtered2);
    cropBox.filter(*cloud_filtered3);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered3;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());
    for (int index : inliers->indices)
        plane_cloud->points.push_back(cloud->points[index]);

    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Set to true to extract the outliers (points that don't match the model)

    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // // Create the segmentation object
    // typename pcl::SACSegmentation<PointT> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (maxIterations);
    // seg.setDistanceThreshold (distanceThreshold);

    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);

	// std::cout << "Inliers within distance tolerance:" << inliers->indices.size() << std::endl;
    Ransac<PointT>(*inliers, cloud, maxIterations, distanceThreshold);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void proximity(typename pcl::PointCloud<PointT>::ConstPtr points, int idx, pcl::PointIndices& cluster, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol)
{
	processed[idx] = true;
	cluster.indices.push_back(idx);
	std::vector<int> proxPoint = tree->search((*points)[idx], distanceTol);

	for (int i : proxPoint)
		if (!processed[i])
			proximity(points, i, cluster, processed, tree, distanceTol);
}

template<typename PointT>
void euclideanCluster(typename pcl::PointCloud<PointT>::ConstPtr points, std::vector<pcl::PointIndices> &clusters, KdTree<PointT>* tree, int minSize, int maxSize, float distanceTol)
{
	std::vector<bool> processed(points->size(), false);	
	
	for (size_t i = 0; i < points->size(); ++i)
	{
		if (processed[i])
			continue;
		std::cout << "Creating cluster from point " << i << std::endl;
		pcl::PointIndices cluster;
		proximity(points, i, cluster, processed, tree, distanceTol);
        if (cluster.indices.size() >= minSize && cluster.indices.size() <= maxSize)
		    clusters.push_back(std::move(cluster));
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    std::vector<pcl::PointIndices> cluster_indices;

    // Creating the KdTree object for the search method of the extraction
    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (cloud);

    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance (clusterTolerance); // 2cm
    // ec.setMinClusterSize (minSize);
    // ec.setMaxClusterSize (maxSize);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud);
    // ec.extract (cluster_indices);
    KdTree<PointT>* tree = new KdTree<PointT>;
  
    for (int i=0; i<cloud->size(); i++) 
    	tree->insert((*cloud)[i],i); 
  	euclideanCluster<PointT>(cloud, cluster_indices, tree, minSize, maxSize, clusterTolerance);
    
    for (const auto& cluster : cluster_indices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for (const auto& idx : cluster.indices) {
        cloud_cluster->push_back((*cloud)[idx]);
      }
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      clusters.push_back(cloud_cluster);
     }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}