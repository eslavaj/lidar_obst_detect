/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

#include <pcl/filters/random_sample.h>

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sample(new pcl::PointCloud<pcl::PointXYZ>());


	pcl::RandomSample<pcl::PointXYZ> sampler;
	sampler.setInputCloud(cloud);
	sampler.setSample(2);

	float A_most, B_most, C_most;
	int nbr_inliers_most = 0;

	// For max iterations
	for(int iter=0; iter<maxIterations; iter++)
	{
		// Randomly sample subset and fit line
		sampler.setSeed(random());
		sampler.filter(*cloud_sample);
		pcl::PointXYZ p1 = cloud_sample->points[0];
		float x1 = p1.x;
		float y1 = p1.y;
		pcl::PointXYZ p2 = cloud_sample->points[1];
		float x2 = p2.x;
		float y2 = p2.y;
		/*Calculate A, B, C*/
		float A = y1-y2;
		float B = x2-x1;
		float C = x1*y2 - x2*y1;
		int nbr_inliers=0;
		std::unordered_set<int> inliers_tmp;

		// Measure distance between every point and fitted line
		for(int index=0; index< cloud->size(); index++)
		{
			pcl::PointXYZ p = cloud->points[index];
			float x = p.x;
			float y = p.y;
			float d = fabs(A*x + B*y +C)/(sqrt(A*A+B*B));

			// If distance is smaller than threshold count it as inlier
			if (d<=distanceTol)
			{
				nbr_inliers++;
				inliers_tmp.insert(index);
			}

			if (nbr_inliers>nbr_inliers_most)
			{
				A_most = A;
				B_most = B;
				C_most = C;
				nbr_inliers_most = nbr_inliers;
				inliersResult = inliers_tmp;
			}

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
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.3);

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
