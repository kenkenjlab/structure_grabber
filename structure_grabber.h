/**
 * Grabber for Structure Sensor (Depth and Infrared) using OpenNI2
 */

#ifndef STRUCTURE_GRABBER_VERSION
#define STRUCTURE_GRABBER_VERSION 2014072601

// ------------------------------------------
//		Includes
// ------------------------------------------

#include "oni2_grabber.h"

// STL
#include <iostream>

// ------------------------------------------
//		Classes
// ------------------------------------------

class StructureGrabber : public Oni2Grabber {
private:
	//// Private Fields ////
	double min_distance_, max_distance_;

	//// Private Methods ////
	unsigned char convertDepth2UChar_(unsigned short depth) const;
	static unsigned char convertInfrared2UChar_(unsigned short brightness);

public:
	//// Public Methods ////
	StructureGrabber() : Oni2Grabber(), min_distance_(0.30), max_distance_(5.0) {}
	~StructureGrabber() {}

	// Setter
	inline void setDepthRange(double min_distance, double max_distance) { min_distance_ = min_distance; max_distance_ = max_distance; }

	// Getter
	inline double getDepthRangeMin() const { return min_distance_; }
	inline double getDepthRangeMax() const { return max_distance_; }

	// Processing
	int copyDepthDataTo(unsigned short* depth_data) const;
	int copyDepthDataTo(unsigned char* depth_data) const;
	int copyInfraredDataTo(unsigned short* infrared_data) const;
	int copyInfraredDataTo(unsigned char* infrared_data) const;
	int copyColorDataTo(unsigned short* color_data) const { std::cerr << "*copyColorDataTo is not available!" << std::endl; abort(); return -1; }
	int copyColorDataTo(unsigned char* color_data) const { std::cerr << "*copyColorDataTo is not available!" << std::endl; abort(); return -1; }

#ifdef OPENNI2_GRABBER_USE_OPENCV
public:
	int copyDepthImageTo(IplImage **image) {
		if(*image == nullptr)
			*image = cvCreateImage(cvSize(getWidth(), getHeight()), IPL_DEPTH_8U, 1);

		return copyDepthDataTo(reinterpret_cast<unsigned char*>((*image)->imageData));
	}
	int copyInfraredImageTo(IplImage **image) {
		if(*image == nullptr)
			*image = cvCreateImage(cvSize(getWidth(), getHeight()), IPL_DEPTH_8U, 1);

		return copyInfraredDataTo(reinterpret_cast<unsigned char*>((*image)->imageData));
	}
#endif	// End of #ifdef OPENNI2_GRABBER_USE_OPENCV

#ifdef OPENNI2_GRABBER_USE_OPENCV2
public:
	int copyDepthImageTo(cv::Mat &image) {
		if(image.cols != getWidth() || image.rows != getHeight())
			image = cv::Mat(cv::Size(getWidth(), getHeight()), CV_8UC1, cv::Scalar::all(0));
		return copyDepthDataTo(reinterpret_cast<unsigned char*>(image.data));
	}
	int copyInfraredImageTo(cv::Mat &image) {
		if(image.cols != getWidth() || image.rows != getHeight())
			image = cv::Mat(cv::Size(getWidth(), getHeight()), CV_8UC1, cv::Scalar::all(0));
		return copyInfraredDataTo(reinterpret_cast<unsigned char*>(image.data));
	}
#endif	// End of #ifdef OPENNI2_GRABBER_USE_OPENCV2

#ifdef OPENNI2_GRABBER_USE_PCL
public:
	template <class PointType> int copyPointCloudTo(pcl::PointCloud<PointType> &cloud) const;
#endif	// End of #ifdef OPENNI2_GRABBER_USE_PCL

};


// ------------------------------------------
//		Template Methods
// ------------------------------------------

#ifdef OPENNI2_GRABBER_USE_PCL

template <class PointType>
int StructureGrabber::copyPointCloudTo(pcl::PointCloud<PointType> &cloud) const {
	// (1) Check
	if(!isOpened())
		return 1;
	if(!isDepthDataAvailable())
		return 2;

	// (2) Allocate cloud
	cloud.width = getWidth();
	cloud.height = getHeight();
	cloud.is_dense = true;
	cloud.resize(getSize());

	// (3) Acquire raw depth data
	const unsigned short* raw_depth_data_ptr = getDepthDataPtr();

	// (4) Transform into PCL coordinate
	for(int y = 0, i = 0; y < cloud.height; y++) {
		for(int x = 0; x < cloud.width; x++, i++) {
			// (4-1) Convert
			PointType &p = cloud.points[i];
			getPointFromDepth(x, y, p.x, p.y, p.z);
			p.x *= -1;
			p.y *= -1;
			
			// (4-2) Adjust to PCL coordinate
			p.getVector3fMap() /= 1000;
		}
	}
	return 0;
}

#endif	// End of #ifdef OPENNI2_GRABBER_USE_PCL



#endif	// End of #ifndef STRUCTURE_GRABBER_VERSION