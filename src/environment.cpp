/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    /*Create lidar sensor*/
    Lidar * mylidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = mylidar->scan();

    /*Display Lidar rays*/
    //renderRays(viewer, mylidar->position, inputCloud);

    /*Display the whole Point cloud*/
    //renderPointCloud(viewer, inputCloud, "inputCloud");


    /*Create point processor*/
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);

    /*Display a segment cloud and a obstacles cloud*/
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,1,1));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));


    /*Segment the obstacle cloud in several clusters*/
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);


    /*Display the different clusters each one with a different color, here we assume there is only 3 clusters in the obstacle cloud*/
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessor->numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          /*Display bounding boxes*/
          Box box = pointProcessor->BoundingBox(cluster);
          renderBox(viewer,box,clusterId);

          ++clusterId;
    }


}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");

  /*Filtering point cloud and applying the RoI*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25 , Eigen::Vector4f (-10, -5.5, -2, 1), Eigen::Vector4f ( 28, 7, 5, 1));
  //renderPointCloud(viewer,filterCloud,"filterCloud");

  /*Segment plane and abstacles*/
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 50, 0.2);

  /*Display a segment cloud and a obstacles cloud*/
  //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,1,1));
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));


  /*Segment the obstacle cloud in several clusters*/
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstClusters = pointProcessorI->Clustering(segmentCloud.first, 0.6, 20, 13000);

  /*Display the different clusters each one with a different color, here we assume there is only 3 clusters in the obstacle cloud*/
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
	  //renderBox(viewer,box,clusterId, colors[clusterId%(colors.size())]);
	  renderBox(viewer,box,clusterId, Color(1,0,0));

	  ++clusterId;
  }


}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    /*
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
    */


    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    //cityBlock(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped ())
    {

    	// Clear viewer
    	viewer->removeAllPointClouds();
    	viewer->removeAllShapes();

    	// Load pcd and run obstacle detection process
    	inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    	cityBlock(viewer, pointProcessorI, inputCloudI);

    	streamIterator++;
    	if(streamIterator == stream.end())
    		streamIterator = stream.begin();

    	viewer->spinOnce ();
    }



}
