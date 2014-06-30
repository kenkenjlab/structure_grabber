// ------------------------------------------
//		Includes ((>ƒÖ<))
// ------------------------------------------

#include "structure_grabber.h"

// ------------------------------------------
//		Public Methods ((>ƒÖ<))
// ------------------------------------------

// Copy raw data of depth
int StructureGrabber::copyDepthDataTo(unsigned short* depth_data) const {
	if(!isOpened())
		return 1;
	if(!isDepthDataAvailable())
		return 2;
	std::memcpy(depth_data, getDepthDataPtr(), getSize() * sizeof(unsigned short));
	return 0;
}

// Copy trimmed data of depth
int StructureGrabber::copyDepthDataTo(unsigned char* depth_data) const {
	if(!isOpened())
		return 1;
	if(!isDepthDataAvailable())
		return 2;
	const unsigned short *depth_data_ptr = getDepthDataPtr();
	for(int i = 0; i < getSize(); i++)
		depth_data[i] = convertDepth2UChar_(depth_data_ptr[i]);
	return 0;
}

// Copy raw data of infrared
int StructureGrabber::copyInfraredDataTo(unsigned short* infrared_data) const {
	if(!isOpened())
		return 1;
	if(!isInfraredAvailable())
		return 2;
	std::memcpy(infrared_data, getInfraredDataPtr(), getSize() * sizeof(unsigned short));
	return 0;
}

// Copy trimmed data of infrared
int StructureGrabber::copyInfraredDataTo(unsigned char* infrared_data) const {
	if(!isOpened())
		return 1;
	if(!isInfraredAvailable())
		return 2;
	const unsigned short *infrared_data_ptr = getInfraredDataPtr();
	for(int i = 0; i < getSize(); i++)
		infrared_data[i] = convertInfrared2UChar_(infrared_data_ptr[i]);
	return 0;
}


// ------------------------------------------
//		Private Methods ((>ƒÖ<))
// ------------------------------------------

// Convert depth from unsigned short (16bit) to unsigned char (8bit)
unsigned char StructureGrabber::convertDepth2UChar_(unsigned short depth) const {
	// depth in unsignd short is described by milimeter (eg. 4731 means 4.731m)
	float gray(0.0);
	float distance = (float)depth / 1000;	// meter
	if(distance > min_distance_) {
		if(distance < max_distance_) {
			gray = distance / max_distance_ * 255;
		} else {
			// In the case of "too far"
			gray = 255;
		}
	} else {
		// In the case of "too close"
		gray = 0;
	}
	return gray;
}

// Convert infrared from unsigned short (16bit) to unsigned char (8bit)
unsigned char StructureGrabber::convertInfrared2UChar_(unsigned short brightness) {
	float gray(0.0);
	float ratio = (float)brightness / 1024;	// The upper limit of infrared brightness value seems to be 1024
	if(ratio > 1.0) {
		gray = 255;
	} else {
		gray = ratio * 255;
	}
	return gray;
}
