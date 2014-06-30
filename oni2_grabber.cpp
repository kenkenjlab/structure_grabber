// ------------------------------------------
//		Includes ((>É÷<))
// ------------------------------------------

#include "oni2_grabber.h"

// STL
#include <iostream>


// ------------------------------------------
//		Public Methods ((>É÷<))
// ------------------------------------------

// Open and Initialize OpenNI2 device on USB Port
int Oni2Grabber::open(const char* uri) {
	// (1) Check
	if(isOpened()) {
		if(isDebugMode())
			std::cout << " Already opened." << std::endl;
		return -2;
	}
	
	// (2) Open device on USB port 
	try {
		openni::OpenNI::initialize();
		if(device_.open(uri ? uri : openni::ANY_DEVICE)) {	// Open a first-found camera
			if(isDebugMode())
				std::cout << " Opening failed, OpenNI2 device connected?" << std::endl;
			return -1;
		}

		if(enable_depth_)
			createAndStartStream_(depth_stream_, openni::SENSOR_DEPTH);

		if(enable_infrared_)
			createAndStartStream_(infrared_stream_, openni::SENSOR_IR);

		if(enable_color_)
			createAndStartStream_(color_stream_, openni::SENSOR_COLOR);

	} catch(std::exception &) {
		std::cerr << openni::OpenNI::getExtendedError() << std::endl;
	}
	
	// (3) Set parameters
	if(isDebugMode())
		std::cout << " Successfully opened." << std::endl;
	setOpened_();
	initCamera_();
	return 0;
}
 
// Close OpenNI2 device
int Oni2Grabber::close() {
	// (1) Check
	if(!isOpened())
		return 1;

	// (2) Close camera
	try {
		if(enable_depth_)
			stopAndDestroyStream_(depth_stream_);
		if(enable_infrared_)
			stopAndDestroyStream_(infrared_stream_);
		if(enable_color_)
			stopAndDestroyStream_(color_stream_);
		streams_.clear();
		device_.close();
	} catch(std::exception &) {
		std::cerr << openni::OpenNI::getExtendedError() << std::endl;
	}
    
	// (3) Set parameters
	if(isDebugMode())
		std::cout << " Successfully closed." << std::endl;
	setClosed_();
	return 0;
}

// Update Image
int Oni2Grabber::acquire() {
	// (1) Check
	if(!isOpened())
		return 1;

	// (2) Acquire new data
	int changed_index;
	openni::OpenNI::waitForAnyStream(&streams_[0], streams_.size(), &changed_index);
	switch(changed_index) {
	case 0:
		if(enable_depth_) {
			acquireNewFrame_(changed_index, depth_frame_ref_);
		} else if(enable_infrared_) {
			acquireNewFrame_(changed_index, infrared_frame_ref_);
		} else if(enable_color_) {
			acquireNewFrame_(changed_index, color_frame_ref_);
		}
		break;
	case 1:
		if(enable_infrared_) {
			acquireNewFrame_(changed_index, infrared_frame_ref_);
		} else if(enable_color_) {
			acquireNewFrame_(changed_index, color_frame_ref_);
		}
		break;
	case 2:
		if(enable_color_) {
			acquireNewFrame_(changed_index, color_frame_ref_);
		}
		break;
	}

	return 0;
}

// Start recording to file
int Oni2Grabber::startRecording(const char *uri) {
	if(isRecording())
		return 1;	// Not stopped
	try {
		recording_file_name_ = uri;
		recorder_.create(uri);
		for(std::vector<openni::VideoStream*>::iterator it = streams_.begin(); it != streams_.end(); it++)
			recorder_.attach(**it);
		recorder_.start();
	} catch(std::exception &) {
		std::cerr << openni::OpenNI::getExtendedError() << std::endl;
	}
	return 0;
}

// Stop recording to file
int Oni2Grabber::stopRecording() {
	if(!isRecording())
		return 1;	// Not started
	try {
		recorder_.stop();
		recorder_.destroy();
	} catch(std::exception &) {
		std::cerr << openni::OpenNI::getExtendedError() << std::endl;
	}
	recording_file_name_.clear();
	return 0;
}



// ------------------------------------------
//		Private Methods ((>É÷<))
// ------------------------------------------

// Prepare parameters
void Oni2Grabber::initCamera_() {
	auto video_mode = streams_[0]->getVideoMode();

	// (1) Set parameters
	if(desired_width_ && desired_height_) {
		video_mode.setResolution(640,480);
		streams_[0]->setVideoMode(video_mode);
	}
	
	// (2) Get parameters
	width_ = video_mode.getResolutionX();
	height_ = video_mode.getResolutionY();
	fps_ = video_mode.getFps();
	size_ = width_ * height_;
}

// Start recording
void Oni2Grabber::createAndStartStream_(openni::VideoStream &stream, openni::SensorType sensor_type) {
	stream.create(device_, sensor_type);
	stream.start();
	streams_.push_back(&stream);
}

// Stop recording
void Oni2Grabber::stopAndDestroyStream_(openni::VideoStream &stream) {
	stream.stop();
	stream.destroy();
}

