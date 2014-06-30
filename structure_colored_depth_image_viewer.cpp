// ------------------------------------------
//		Includes ((>É÷<))
// ------------------------------------------

#include <iostream>
#include <opencv2/opencv.hpp>
#include "structure_grabber.h"	// Include OpenCV headers before this inclusion if you want to capture images

// ------------------------------------------
//		Prototypes ((>É÷<))
// ------------------------------------------

void createColoredDepthImage(const unsigned short *input, cv::Mat &output, int width, int height);
cv::Vec3b convertDepth2RGB(float distance);
void convertHSV2RGB(float h, float s, float v, float &r, float &g, float &b);

// ------------------------------------------
//		Entry Point ((>É÷<))
// ------------------------------------------

int main(int argc, char** argv) {
	// (1) Open OpenNI2 device
	StructureGrabber grabber;
	grabber.setDebugMode();
	grabber.enableDepth();
	grabber.enableInfrared();
	grabber.disableColor();
	grabber.setDepthRange(0.3, 5.0);
	if(argc > 1) {
		// (1-A) Open a file
		const std::string file_path(argv[1]);
		if(grabber.open(file_path)) {
			std::cerr << "Faled to open " << file_path << std::endl;
			return -1;
		}
	} else {
		// (1-B) Open a real device
		if(grabber.open()) {
			std::cerr << "Faled to open OpenNI2 device" << std::endl;
			return -2;
		}
	}

	// (2) Prepare for images
	cv::Size image_size = cv::Size(grabber.getWidth(), grabber.getHeight());
	cv::Mat gray_depth_image(image_size, CV_8UC1, cv::Scalar::all(0));
	cv::Mat colored_depth_image(image_size, CV_8UC3, cv::Scalar::all(0));
	cv::Mat infrared_image(image_size, CV_8UC1, cv::Scalar::all(0));
	
	// (3) Iterate
	int key(0);
	while(key != 'q' && key != 27) {	// Break if q or ESC was pressed
		// (3-1) Acquire new frame
		grabber.acquire();

		// (3-2) Create colored depth image
		if(grabber.isDepthDataAvailable()) {
			const unsigned short *depth_data_ptr = grabber.getDepthDataPtr();
			createColoredDepthImage(depth_data_ptr, colored_depth_image, image_size.width, image_size.height);
		}

		// (3-3) Flip images
		cv::flip(colored_depth_image, colored_depth_image, 1);

		// (3-4) Visualize
		cv::imshow("Depth [Colored]", colored_depth_image);
		key = cv::waitKey(1);
	}

	// (4) Close OpenNI2 device
	grabber.close();
	return 0;
}

// ------------------------------------------
//		Functions
// ------------------------------------------

void createColoredDepthImage(const unsigned short *input, cv::Mat &output, int width, int height) {
	for(int y = 0, i = 0; y < height; y++) {
		for(int x = 0; x < width; x++, i++) {
			float distance = (float)input[i] / 1000;	// meter
			output.at<cv::Vec3b>(y, x) = convertDepth2RGB(distance);
		}
	}
}

cv::Vec3b convertDepth2RGB(float distance) {
	cv::Vec3f color_f;	// BGR
	if(distance > 0.3) {
		if(distance < 5.0) {
			convertHSV2RGB(315 - 360.0 * (1 - distance / 5.0), 1.0, 1.0, color_f[2], color_f[1], color_f[0]);
		} else {
			// In the case of "too far"
			color_f = cv::Vec3f(0.5, 0.5, 0.5);
		}
	} else {
		// In the case of "too close"
		color_f = cv::Vec3f(0.0, 0.0, 0.0);
	}

	cv::Vec3b color_b;
	for(int i = 0; i < 3; i++)
		color_b[i] = color_f[i] * 255;
	return color_b;
}

void convertHSV2RGB(float h, float s, float v, float &r, float &g, float &b) {
	// H: 0.0~360.0(deg), S: 0.0~1.0, V: 0.0~1.0
	// (Cited: http://openframeworks.jp/forum/topic.php?id=16)
    int in;
    float fl, m, n;
	while(true) {
		if(h > 360) {
			h -= 360;
		} else if(h < 0) {
			h += 360;
		} else {
			break;
		}		
	}
    in = (int)floor( h / 60 );
    fl = ( h / 60 ) - in;
    if( !(in & 1)) fl = 1 - fl; // if i is even
 
    m = s * ( 1 - s );
    n = v * ( 1 - s * fl );
    switch( in ){
       case 0: r = v; g = n; b = m; break;
       case 1: r = n; g = v; b = m; break;
       case 2: r = m; g = v; b = n; break;
       case 3: r = m; g = n; b = v; break;
       case 4: r = n; g = m; b = v; break;
       case 5: r = v; g = m; b = n; break;
    }
}