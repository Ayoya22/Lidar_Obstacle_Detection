#include <iostream>
#include "render/render.h"
#include "ClusterEngine/ClusterEngine.h"
#include "SegmentEngine/SegmentEngine.h"
#include "processPointClouds/processPointClouds.h"
#include "processPointClouds/processPointClouds.cpp"



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



void lidarProj(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.25, {-10, -5, -3,1}, {28, 7, 1,1});
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0)); 

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 10, 800);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
      Box box = pointProcessorI->BoundingBox(cluster);
      renderBox(viewer, box, clusterId);
      ++clusterId;
      
  }

}


int main(int argc, char** argv)
{
	std::cout << "starting enviroment" << std::endl;


	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);


    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../pcd/data_1");

    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
	    // Clear viewer
	    viewer->removeAllPointClouds();
	    viewer->removeAllShapes();

	    // Load pcd and run obstacle detection process
    	inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    	lidarProj(viewer, pointProcessorI, inputCloudI);

    	streamIterator++;
    	if(streamIterator == stream.end())
        	streamIterator = stream.begin();

    	viewer->spinOnce ();
    }


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 

}