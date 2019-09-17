/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"


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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	float A_most, B_most, C_most, D_most;
	int nbr_inliers_most = 0;
	size_t cloud_size = cloud->size();

	// For max iterations
	for(int iter=0; iter<maxIterations; iter++)
	{
		/* get 3 index randomly */
		int pindex1 = random()%cloud_size;
		int pindex2 = random()%cloud_size;
		int pindex3 = random()%cloud_size;

		/*It is almost improbable but just in case let's check points are different*/
		if( (pindex1==pindex2) || (pindex2==pindex3) || (pindex1==pindex3))
		{
			break;
		}

		/*The 3 points*/
		pcl::PointXYZ p1 = cloud->points[pindex1];
		pcl::PointXYZ p2 = cloud->points[pindex2];
		pcl::PointXYZ p3 = cloud->points[pindex3];

		/*Vector from origin to p1*/
		Eigen::Vector3f v1 = p1.getVector3fMap();
		/*Vector from p1 to p2*/
		Eigen::Vector3f v12 = p2.getVector3fMap() - v1;
		/*Vector from p1 to p3*/
		Eigen::Vector3f v13 = p3.getVector3fMap() - v1;
		/*Normal vector of plane*/
		Eigen::Vector3f normal_vect = v12.cross(v13);

		int nbr_inliers=0;
		std::unordered_set<int> inliers_tmp;

		/*Measure distance between every point and fitted line*/
		for(int index=0; index< cloud->size(); index++)
		{
			pcl::PointXYZ p = cloud->points[index];
			Eigen::Vector3f vp = p.getVector3fMap();
			float d = fabs(normal_vect.dot(vp) - normal_vect.dot(v1)) / normal_vect.norm();

			/*If distance is smaller than threshold count it as inlier*/
			if (d<=distanceTol)
			{
				nbr_inliers++;
				inliers_tmp.insert(index);
			}
		}

		if (nbr_inliers>nbr_inliers_most)
		{
			nbr_inliers_most = nbr_inliers;
			inliersResult = inliers_tmp;
		}
	}
	
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.35);

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
