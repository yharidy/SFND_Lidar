// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "quiz/cluster/kdtree.h"
//#include <pcl/filters/crop_box.h>


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
	// Create the filtering object
	pcl::VoxelGrid<PointT> sor;
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	sor.setInputCloud(cloud);
	sor.setLeafSize(filterRes, filterRes, filterRes);
	sor.filter(*cloud_filtered);

	// Create  ROI
	pcl::CropBox<PointT> roi(true);
	roi.setInputCloud(cloud_filtered);
	roi.setMin(minPoint);
	roi.setMax(maxPoint);
	typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);
	roi.filter(*cloud_region);

	std::vector<int> indices;

	pcl::CropBox<PointT> roof(true);
	roof.setInputCloud(cloud_region);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	for (int point : indices)
		inliers->indices.push_back(point);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_region);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());
	typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());
	
	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;

	for (int i : inliers->indices) 
		planeCloud->points.push_back(cloud->points[i]);
	
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>	
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	// Time segmentation process
	/*
	auto startTime = std::chrono::steady_clock::now();
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers{ new pcl::PointIndices() };
	pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
	
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIterations);
	seg.setDistanceThreshold(distanceThreshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
	*/
	auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
	pcl::PointIndices::Ptr inliersResult{ new pcl::PointIndices };
	srand(time(NULL));

	while (maxIterations--) {

		while (inliers->indices.size() < 3)
			inliers->indices.push_back(rand() % (cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto itr = inliers->indices.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float v1[3] = { x2 - x1, y2 - y1, z2 - z1 };
		float v2[3] = { x3 - x1, y3 - y1, z3 - z1 };
		float v3[3] = { (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1), (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1), (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1) };

		float A = v3[0];
		float B = v3[1];
		float C = v3[2];

		float D = -(A*x1 + B * y1 + C * z1);

		// Measure distance between every point and fitted line
		for (int index = 0; index < cloud->points.size(); index++) {
			
				
			if (std::find(inliers->indices.begin(), inliers->indices.end(), index) != inliers->indices.end())
				continue;

			PointT point = cloud->points[index];
			float d = std::fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);
			// If distance is smaller than threshold count it as inlier
			if (d < distanceThreshold)
				inliers->indices.push_back(index);

		}
		if (inliers->indices.size() > inliersResult->indices.size())
			inliersResult = inliers;
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took " << elapsedTime.count() << "milliseconds" << endl;
	// Return indicies of inliers from fitted line with most inliers
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult, cloud);
	return segResult;
}
/*
void clusterHelper(const std::vector<std::vector<float>>& points, int& i, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float& distanceTol)
{
	processed[i] = true;
	cluster.push_back(i);
	std::vector<float> point = points[i];
	std::vector<int> nextPts = tree->search(point, distanceTol);

	for (int id : nextPts)
	{
		if (!processed[id])
			clusterHelper(points, id, cluster, processed, tree, distanceTol);
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(), false);
	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;

		clusterHelper(points, i, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;

	}

	return clusters;

}
*/
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
	
	std::vector<pcl::PointIndices> clusters_indices;
	// Create Kd-tree
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	// Create clustering object
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(clusterTolerance); // 2cm
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(clusters_indices);

	// Perform clustering 
	int j = 0;

	for (std::vector<pcl::PointIndices>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it) {
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (const auto& idx : it->indices)
			cloud_cluster->push_back((*cloud)[idx]);
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
	
	/*
	KdTree* tree = new KdTree;
	for (int i = 0; i < cloud->points.size(); i++)
		tree->insert(cloud->points[i], i);

	std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud->points, tree, 3.0);

	int i = 0;
	while (i < cluster_indices.size())
	{
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		std::vector<int> indices = cluster_indices[i];
		for (const auto& idx : indices)
			cloud_cluster->push_back((*cloud)[idx]);
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		clusters.push_back(cloud_cluster);
		i++;
	}
	*/
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