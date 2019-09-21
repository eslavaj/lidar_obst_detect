/**
 *  @brief The main file
 *
 *  This application shows does filtering, plane segmentation and obstacle clustering
 *  using as input real data in point cloud format.
 *
 */

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


/**
 * @brief Process the input cloud and render the whole frame.
 *
 * This functions call the processing functions and renders the plane and obstacles cloud and the obstacle boxes.
 *
 * @param viewer[out]: the viewer.
 * @param pointProcessorI[in,out]: Object to process points.
 * @param pointProcessorI[in]: The input point cloud.
 *
 * @return none.
 */
void process_cloud(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

  /*Filtering point cloud and applying the RoI*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25 , Eigen::Vector4f (-10, -5.5, -2, 1), Eigen::Vector4f ( 28, 7, 5, 1));

  /*Segment plane and abstacles*/
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 50, 0.2);

  /*Display the plane cloud*/
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  /*If you want to display obstacle cloud before obstacle clustering then add
   * the following line, this is useful to verify obstacle clustering:
   *
   * renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,1,1));
   * */

  /*Segment the obstacle cloud in several clusters, one for every obstacle*/
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstClusters = pointProcessorI->Clustering(segmentCloud.first, 0.6, 20, 13000);

  /*Display the different clusters and their boxes*/
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1), Color(1,1,0), Color(1,0,1), Color(0.8,0.7,0.4), Color(0.8,0.2,0.2)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstClusters)
  {
	  /*Render obstacles clusters*/
	  std::cout << "cluster size ";
	  pointProcessorI->numPoints(cluster);
	  renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%(colors.size())]);

	  /*Display bounding boxes for each obstacle cluster*/
	  Box box = pointProcessorI->BoundingBox(cluster);
	  renderBox(viewer,box,clusterId, Color(1,0,0));
	  /*If you want to render each obstacle with a different color,
	   * use this line instead:
	   *
	   * renderBox(viewer,box,clusterId, colors[clusterId%(colors.size())]);
	   * */
	  ++clusterId;
  }

}




/**
 * @brief Initializes the camera.
 *
 * @param setAngle[in]: Initial angle.
 * @param viewer[out]: the viewer.
 *
 * @return none.
 */
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);

    /*set camera position and angle*/
    viewer->initCameraParameters();
    /*distance away in meters*/
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}




/**
 *
 * @brief Main function.
 *
 *
 * @return int.
 */
int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    /*Create the viewer*/
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    /*Initialize camera*/
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    /*Create the point processor*/
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    /*Prepare the stream data*/
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
    	/*Clear viewer*/
    	viewer->removeAllPointClouds();
    	viewer->removeAllShapes();

    	/*Load pcd*/
    	inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    	/*Run obstacle detection process*/
    	process_cloud(viewer, pointProcessorI, inputCloudI);

    	streamIterator++;
    	if(streamIterator == stream.end())
    		streamIterator = stream.begin();

    	viewer->spinOnce ();
    }

}



