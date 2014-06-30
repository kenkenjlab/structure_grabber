// ------------------------------------------
//		Includes ((>ƒÖ<))
// ------------------------------------------

#include <iostream>
#include <opencv2/opencv.hpp>
#include "structure_grabber.h"	// Include OpenCV headers before this inclusion if you want to capture images

// ------------------------------------------
//		Entry Point ((>ƒÖ<))
// ------------------------------------------

int main(int argc, char** argv) {
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
	int key(0);
	while(key != 'q' && key != 27) {
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
		key = cv::waitKey(10);

		// (3-6) Save
		switch(key) {
		case 's':
			std::cout << "Saving...";
			cv::imwrite("capture_depth.png", depth_image);
			cv::imwrite("capture_infrared.png", infrared_image);
			std::cout << "done" << std::endl;
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
