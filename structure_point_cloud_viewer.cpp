// ------------------------------------------
//		Includes ((>É÷<))
// ------------------------------------------

#include <iostream>
#include <string>
#include <sstream>

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
int frames_saved = 0;
 

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
	std::stringstream out;
	std::string name;
	if(event.getKeySym() == "s" && event.keyUp()) {
		std::cout << "Saving frame " << frames_saved << "...\n";
                out << frames_saved; 
                name = "InputCloud" + out.str() + ".pcd"; 
		pcl::io::savePCDFile(name, *cloud);
		std::cout << "done" << std::endl;
                frames_saved++;
	}
}
