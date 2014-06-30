// ------------------------------------------
//		Includes ((>É÷<))
// ------------------------------------------

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "structure_grabber.h"	// Include PCL headers before this inclusion if you want to capture point clouds

// ------------------------------------------
//		Prototypes ((>É÷<))
// ------------------------------------------

void initViewer(pcl::visualization::PCLVisualizer &viewer);
void keyboardCallback (const pcl::visualization::KeyboardEvent &event);

// ------------------------------------------
//		Global Variables ((>É÷<))
// ------------------------------------------

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// ------------------------------------------
//		Entry Point ((>É÷<))
// ------------------------------------------

int main(int argc, char** argv) {
	// (1) Open OpenNI2 Device
	StructureGrabber grabber;
	grabber.enableDepth();
	grabber.disableInfrared();
	grabber.disableColor();
	if(argc > 1 ? grabber.open(argv[1]) : grabber.open()) {
		std::cerr << "Faled to open OpenNI2 device" << std::endl;
		return -1;
	}
	
	// (2) Open Viewer
	pcl::visualization::PCLVisualizer viewer("Point Cloud");
	initViewer(viewer);
	viewer.addPointCloud(cloud);

	// (3) Iterate
	int key(0);
	while(!viewer.wasStopped()) {
		// (3-1) Acquire new frame
		grabber.acquire();

		// (3-2) Visualize
		grabber.copyPointCloudTo(*cloud);
		viewer.updatePointCloud(cloud);
		viewer.spinOnce();
	}

	// (4) Close OpenNI2 device
	grabber.close();
	return 0;
}



// ------------------------------------------
//		Functions ((>É÷<))
// ------------------------------------------

void initViewer(pcl::visualization::PCLVisualizer &viewer) {
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(1.0, "reference");
	viewer.initCameraParameters();
	viewer.setRepresentationToPointsForAllActors();
	viewer.setCameraPosition(0, 0, -1, 0, 0, 0, 0, -1, 0);
	viewer.registerKeyboardCallback(keyboardCallback);
}

void keyboardCallback(const pcl::visualization::KeyboardEvent &event) {
	if(event.getKeySym() == "v" && event.keyUp()) {
		std::cout << "Saving...";
		pcl::io::savePCDFile("capture.pcd", *cloud);
		std::cout << "done" << std::endl;
	}
}