/**
 *  @brief class to process point cloud
 *
 *  This class contains the methods needed for processing the point cloud.
 *
 */

#include <unordered_set>
#include "processPointClouds.h"
#include "kdtree.h"


/*By default we use plane segmentation and obstacle clustering algorithms developed from scratch
 * If you want to use PCL library to segment plane and clustering you must define USE_PCL_4_PLANE_CLUSTER
 * */
//#define USE_PCL_4_PLANE_CLUSTER (1)


template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


/**
 * @brief Initial cloud filtering.
 *
 * This method crop the farthest and the roof area, it also downsamples the point cloud.
 *
 * @param cloud[in]: the input cloud to be filtered.
 * @param filterRes[in]: filter resolution for downsampling.
 * @param minPoint[in]: The initial point of the region of interest.
 * @param maxPoint[in]: The final point of the region of interest.
 *
 * @return the filtered cloud.
 */
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    /*Time start filtering*/
    auto startTime = std::chrono::steady_clock::now();

    /*Down-sampling the cropped cloud*/
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxelg;
    voxelg.setInputCloud (cloud);
    voxelg.setLeafSize (filterRes, filterRes, filterRes);
    voxelg.filter (*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr roiCloud(new pcl::PointCloud<PointT>);
    /*We crop out the farthest region*/
    pcl::CropBox<PointT> region_tmp(true);
    region_tmp.setMin(minPoint);
    region_tmp.setMax(maxPoint);
    region_tmp.setInputCloud(filteredCloud);
    region_tmp.filter(*roiCloud);

    /*roof_indices will store the indices of points of the roof*/
    std::vector<int> roof_indices;
    pcl::CropBox<PointT> region_roof(true);
    region_tmp.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    region_tmp.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    region_tmp.setInputCloud(roiCloud);
    region_tmp.filter(roof_indices);

    /*change the type to be used by ExtractIndices class*/
    pcl::PointIndices::Ptr roof_points{new pcl::PointIndices};
    for(auto index : roof_indices)
    {
    	roof_points->indices.push_back(index);
    }

    /*Extract the roof points from cloud*/
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(roiCloud);
    extract.setIndices(roof_points);
    extract.setNegative(true);
    extract.filter(*roiCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roiCloud;

}

/**
 * @brief Separate one cloud on two clouds according to a set of indices.
 *
 * This method takes one cloud and a set of indices of points and returns two clouds.
 * The first cloud contains the points that are not listed in the indices set.
 * The second cloud contains the points that are listed in the indices set.
 *
 * @param inliers[in]: the set of indices.
 * @param cloud[in]: the input cloud to be split.
 *
 * @return a pair of two clouds.
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{

	typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloud_o(new pcl::PointCloud<PointT>());

	for(int index : inliers->indices)
	{
		cloud_p->points.push_back(cloud->points[index]);
	}

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (*cloud_o);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_o, cloud_p);
    return segResult;
}



#ifdef USE_PCL_4_PLANE_CLUSTER
/**
 * @brief Segment a cloud in two clouds: the plane cloud and the obstacles cloud, this version use the PCL library.
 *
 * @param cloud[in]: the input cloud to be split.
 * @param maxIterations[in]: max number of iterations for RANSAC algorithm.
 * @param distanceThreshold[in]: the distance threshold to consider a point as part of plane.
 *
 * @return a pair of two clouds, the first is the obstacles cloud and the second is the plane cloud.
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();


    pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (maxIterations);
	seg.setDistanceThreshold (distanceThreshold);

	seg.setInputCloud(cloud);
	seg.segment (*inliers, *coefficients);
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
#else


/**
 * @brief Segment a cloud in two clouds: the plane cloud and the obstacles cloud, this version uses algorithms implemented from scratch.
 *
 * @param cloud[in]: the input cloud to be split.
 * @param maxIterations[in]: max number of iterations for RANSAC algorithm.
 * @param distanceThreshold[in]: the distance threshold to consider a point as part of plane.
 *
 * @return a pair of two clouds, the first is the obstacles cloud and the second is the plane cloud.
 */
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
	auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
	int nbr_inliers_most = 0;
	Eigen::Vector3f most_normal_vect;
	Eigen::Vector3f most_v1;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
	float d;

	PointT p1, p2, p3;
	int pindex1, pindex2, pindex3;
	Eigen::Vector3f v1, v12, v13, normal_vect;
	Eigen::Vector3f vp;
	PointT p;
	float distanceThreshold_n;


	typename pcl::PointCloud<PointT>::Ptr down_sampled_Cloud(new pcl::PointCloud<PointT>);
	pcl::VoxelGrid<PointT> voxelg;
	voxelg.setInputCloud (cloud);
	voxelg.setLeafSize (1.0, 1.0, 1.0);
	voxelg.filter (*down_sampled_Cloud);

	int cloud_size = down_sampled_Cloud->size();

	boost::random::mt19937 rng;
	boost::random::uniform_int_distribution<> idx_gen(0,cloud_size-1);

	// For max iterations
	for(int iter=0; iter<maxIterations; iter++)
	{
		/* get 3 index randomly */
		/*
		pindex1 = random()%cloud_size;
		pindex2 = random()%cloud_size;
		pindex3 = random()%cloud_size;
		*/
		pindex1 = idx_gen(rng);
		pindex2 = idx_gen(rng);
		pindex3 = idx_gen(rng);

		/*It is almost improbable but just in case let's check points are different*/
		if( (pindex1==pindex2) || (pindex2==pindex3) || (pindex1==pindex3))
		{
			continue;
		}

		/*The 3 points*/
		p1 = down_sampled_Cloud->points[pindex1];
		p2 = down_sampled_Cloud->points[pindex2];
		p3 = down_sampled_Cloud->points[pindex3];

		/*Vector from origin to p1*/
		v1 = p1.getVector3fMap();
		/*Vector from p1 to p2*/
		v12 = p2.getVector3fMap() - v1;
		/*Vector from p1 to p3*/
		v13 = p3.getVector3fMap() - v1;
		/*Normal vector of plane*/
		/*
		normal_vect[0] = v12[1]*v13[2]- v12[2]*v13[1];
		normal_vect[1] = v12[2]*v13[0]- v12[0]*v13[2];
		normal_vect[2] = v12[0]*v13[1]- v12[1]*v13[0];
		 */
		normal_vect = v12.cross(v13);
		distanceThreshold_n = distanceThreshold*normal_vect.norm();
		int nbr_inliers=0;

		/*Measure distance between every point and fitted line*/
		for(int index=0; index< down_sampled_Cloud->size(); index++)
		{
			/*p = down_sampled_Cloud->points[index];
			  vp = p.getVector3fMap();*/
			vp = down_sampled_Cloud->points[index].getVector3fMap();
			d = fabs(normal_vect.dot(vp-v1));

			/*If distance is smaller than threshold count it as inlier*/
			if (d<=distanceThreshold_n)
			{
				nbr_inliers++;
			}
		}

		if (nbr_inliers>nbr_inliers_most)
		{
			nbr_inliers_most = nbr_inliers;
			most_normal_vect = normal_vect;
			most_v1 = v1;
		}
	}

	/*Now search the inliers using the best parameters*/
	distanceThreshold_n = distanceThreshold*most_normal_vect.norm();
	/*using real cloud*/
	cloud_size = cloud->size();
	for(int index=0; index< cloud_size; index++)
	{
		/*p = cloud->points[index];
		  vp = p.getVector3fMap();*/
		vp = cloud->points[index].getVector3fMap();
		d = fabs(most_normal_vect.dot(vp-most_v1));

		/*If distance is smaller than threshold count it as inlier*/
		if (d<=distanceThreshold_n)
		{
			inliers->indices.push_back(index);
		}
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

#endif



#ifdef USE_PCL_4_PLANE_CLUSTER
/**
 * @brief Manage the clustering: the plane cloud and the obstacles cloud, this version use PCL library.
 *
 * @param cloud[in]: the input cloud to be clustered.
 * @param clusterTolerance[in]: the distance threshold to consider a point as part of a cluster.
 * @param minSize[in]: the minimal size of a cluster.
 * @param maxSize[in]: the maximal size of a cluster.
 *
 * @return a vector of point cloud, each point cloud is a cluster.
 */
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	// Time clustering process
	auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	// Creating the KdTree object for the search method of the extraction
	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (clusterTolerance);
	ec.setMinClusterSize (minSize);
	ec.setMaxClusterSize (maxSize);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{

		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			cloud_cluster->points.push_back (cloud->points[*pit]);
		}

		clusters.push_back(cloud_cluster);
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

#else


/**
 * @brief Helper functions to find the points belonging to a cluster by using a KdTree. This version uses algorithms implemented from scratch.
 *
 * @param pointID[in]: the initial point index to start looking its neighbors.
 * @param cluster[in,out]: reference to the cluster to be filled with the founded points.
 * @param processed[in,out]: vector of flags to indicate if a point has been already processed.
 * @param cloud[in]: the input cloud to be clustered.
 * @param tree[in]: the KdTree.
 * @param distanceTol[in]: the distance threshold to consider a point as part of a cluster.
 *
 * @return none.
 */
template<typename PointT>
void proximity(int pointID, typename pcl::PointCloud<PointT>::Ptr& cluster, std::vector<bool>& processed, const typename pcl::PointCloud<PointT>::Ptr& cloud, KdTree<PointT>* tree, float distanceTol)
{

	pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices ());
	cluster_indices->indices.push_back(pointID);

	for(int i=0; i<cluster_indices->indices.size(); i++)
	{
		if( processed[ cluster_indices->indices[i] ] == false)
		{
			processed[ cluster_indices->indices[i] ] = true;
			std::vector<int> clus_tmp = tree->search(cloud->points[ cluster_indices->indices[i] ], distanceTol);
			cluster_indices->indices.insert(cluster_indices->indices.end(), clus_tmp.begin(), clus_tmp.end());
		}
	}

	for(int index : cluster_indices->indices)
	{
		cluster->points.push_back(cloud->points[index]);
	}
}

/**
 * @brief Implement euclidean clustering using a KdTree. This version uses algorithms implemented from scratch.
 *
 * @param cloud[in]: the input cloud to be clustered.
 * @param tree[in]: the KdTree.
 * @param distanceTol[in]: the distance threshold to consider a point as part of a cluster.
 * @param minSize[in]: the minimal size of a cluster.
 * @param maxSize[in]: the maximal size of a cluster.
 *
 * @return a vector of point cloud, each point cloud is a cluster.
 */
template<typename PointT>
typename std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
	/*Vector of clusters to return*/
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	/*to know if a point has been already processed*/
	std::vector<bool> processed(cloud->points.size(), false);

	for(int j = 0; j<cloud->points.size(); j++)
	{
		if(processed[j]==false)
		{
			typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
			proximity(j, cluster, processed, cloud, tree, distanceTol);

			if( (cluster->points.size()>=minSize) && (cluster->points.size()<=maxSize))
			{
				clusters.push_back(cluster);
			}
		}
	}
	return clusters;
}

/**
 * @brief Manage the clustering: the plane cloud and the obstacles cloud, this version uses algorithms implemented from scratch.
 *
 * @param cloud[in]: the input cloud to be clustered.
 * @param clusterTolerance[in]: the distance threshold to consider a point as part of a cluster.
 * @param minSize[in]: the minimal size of a cluster.
 * @param maxSize[in]: the maximal size of a cluster.
 *
 * @return a vector of point cloud, each point cloud is a cluster.
 */
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
	/* Time clustering process*/
	auto startTime = std::chrono::steady_clock::now();

	/*vector of points */
	/*Creating the kdtree object with cloud points*/
	KdTree<PointT> *tree= new KdTree<PointT>(cloud);

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

	delete tree;

    return clusters;
}

#endif



/**
 * @brief Create a box around a cluster.
 *
 * @param cluster[in]: the cluster to be surrounded.
 *
 * @return a point cloud representing the box.
 */
template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    /*Find bounding box for one of the clusters*/
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


/**
 * @brief save a pcd.
 *
 */
template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


/**
 * @brief load a pcd.
 *
 */
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


/**
 * @brief to manage a stream of pcds.
 *
 */
template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
