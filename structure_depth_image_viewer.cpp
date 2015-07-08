// ------------------------------------------
//		Includes ((>É÷<))
// ------------------------------------------

#include <iostream>
#include <string>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "structure_grabber.h"	// Include OpenCV headers before this inclusion if you want to capture images



// ------------------------------------------
//		Entry Point ((>É÷<))
// ------------------------------------------

int main(int argc, char** argv) {


	//Variables
	int frames_saved = 0;
	std::stringstream out;
	std::string name;

	// (1) Open OpenNI2 device
	StructureGrabber grabber;
	grabber.setDebugMode();
	grabber.enableDepth();
	grabber.enableInfrared();
	grabber.disableColor();
	grabber.setDepthRange(0.3, 5.0);
	if(argc > 1 ? grabber.open(argv[1]) : grabber.open()) {
		std::cerr << "Faled to open OpenNI2 device" << std::endl;
		return -1;
	}

	// (2) Prepare for images
	cv::Size image_size = cv::Size(grabber.getWidth(), grabber.getHeight());
	cv::Mat depth_image(image_size, CV_8UC1, cv::Scalar::all(0));
	cv::Mat infrared_image(image_size, CV_8UC1, cv::Scalar::all(0));
	
	// (3) Iterate
	int key;
	while(key != 'q' && key != 27){ // Break if q or ESC was pressed
		// (3-1) Acquire new frame
		grabber.acquire();

		// (3-2) Create depth image
		if(grabber.copyDepthImageTo(depth_image)) {
			std::cerr << "Failed to copy the depth image" << std::endl;
		}

		// (3-3) Create IR image
		if(grabber.copyInfraredImageTo(infrared_image)) {
			std::cerr << "Failed to copy the infrared image" << std::endl;
		}

		// (3-4) Flip images
		cv::flip(depth_image, depth_image, 1);
		cv::flip(infrared_image, infrared_image, 1);

		// (3-5) Visualize
		cv::imshow("Depth", depth_image);
		cv::imshow("Infrared", infrared_image);

		// (3-6) Save
		key = cv::waitKey(1) & 255;
		switch(key){
			case 's': 
				std::cout << "Saving frame " << frames_saved << "...\n";
				out << frames_saved; 
                		name = "capture_depth" + out.str() + ".png"; 
				cv::imwrite(name, depth_image);
                		name = "capture_infrared" + out.str() + ".png"; 
				cv::imwrite(name, infrared_image);
				std::cout << "done" << std::endl;
                                out.str(std::string());
				name.clear();
				frames_saved++;
				break;
			case 'v': 
				std::cout << "Started recording" << std::endl;
				grabber.startRecording("capture.oni");
				break;
			case 'n': 
				std::cout << "Stopped recording" << std::endl;
				grabber.stopRecording();
				break;
			default:
				break;	
				
				
		};
	}
	// (4) Close OpenNI2 device
	grabber.close();
	return 0;
}
