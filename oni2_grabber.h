/**
 * Grabber for the sensors which are compatible with OpenNI2
 */

#ifndef OPENNI2_GRABBER_VERSION
#define OPENNI2_GRABBER_VERSION 2014062901

// ------------------------------------------
//		Includes ((>É÷<))
// ------------------------------------------

// STL
#include <vector>

// OpenNI2
#include <OpenNI.h>


// ------------------------------------------
//		Macros ((>É÷<))
// ------------------------------------------

#ifdef CV_VERSION
#	define OPENNI2_GRABBER_USE_OPENCV
#	if CV_MAJOR_VERSION >= 2
#		define OPENNI2_GRABBER_USE_OPENCV2
#	endif
#endif

#ifdef PCL_VERSION
#	define OPENNI2_GRABBER_USE_PCL
#endif


// ------------------------------------------
//		Classes ((>É÷<))
// ------------------------------------------

class Oni2Grabber {
private:
	//// Private Fields ////
	bool debug_mode_, is_opened_;
	bool enable_depth_, enable_infrared_, enable_color_;
	int width_, height_, size_, fps_, desired_width_, desired_height_;
	std::string recording_file_name_;
	openni::Device device_;
	openni::Recorder recorder_;
	std::vector<openni::VideoStream*> streams_;
	openni::VideoStream depth_stream_, infrared_stream_, color_stream_;
	openni::VideoFrameRef depth_frame_ref_, infrared_frame_ref_, color_frame_ref_;

	//// Private Methods ////
	inline void setOpened_() { is_opened_ = true; }
	inline void setClosed_() { is_opened_ = false; }
	void initCamera_();
	void createAndStartStream_(openni::VideoStream &stream, openni::SensorType sensor_type);
	void stopAndDestroyStream_(openni::VideoStream &stream);
	inline void acquireNewFrame_(openni::VideoStream &stream, openni::VideoFrameRef &frame_ref) { stream.readFrame(&frame_ref); }
	inline void acquireNewFrame_(int stream_index, openni::VideoFrameRef &frame_ref) { acquireNewFrame_(*streams_[stream_index], frame_ref); }

public:
	//// Public Methods ////
	Oni2Grabber() : debug_mode_(false), is_opened_(false), recording_file_name_(), streams_(0),
		enable_depth_(true), enable_infrared_(true), enable_color_(false),
		desired_width_(0), desired_height_(0) {}
	~Oni2Grabber() { close(); }

	// Setter
	inline void setDebugMode(bool mode = true) { debug_mode_ = mode; }
	inline void enableDepth(bool enable = true) { enable_depth_ = enable; }
	inline void enableInfrared(bool enable = true) { enable_infrared_ = enable; }
	inline void enableColor(bool enable = true) { enable_color_ = enable; }
	inline void disableDepth(bool disable = true) { enableDepth(!disable); }
	inline void disableInfrared(bool disable = true) { enableInfrared(!disable); }
	inline void disableColor(bool disable = true) { enableColor(!disable); }
	inline void setResolution(int width, int height) { desired_width_ = width; desired_height_ = height; }

	// Getter
	inline bool isDebugMode() const { return debug_mode_; }
	inline bool isOpened() const { return is_opened_; }
	inline bool isRecording() const { return !recording_file_name_.empty(); }
	inline int getWidth() const { return width_; }
	inline int getHeight() const { return height_; }
	inline int getSize() const { return size_; }
	inline int getFps() const { return fps_; }
	inline bool isDepthDataAvailable() const { return depth_frame_ref_.isValid(); }
	inline bool isInfraredAvailable() const { return infrared_frame_ref_.isValid(); }
	inline bool isColorAvailable() const { return color_frame_ref_.isValid(); }
	inline const unsigned short* getDepthDataPtr() const { return static_cast<const unsigned short*>(depth_frame_ref_.getData()); }
	inline const unsigned short* getInfraredDataPtr() const { return static_cast<const unsigned short*>(infrared_frame_ref_.getData()); }
	inline const unsigned short* getColorDataPtr() const { return static_cast<const unsigned short*>(color_frame_ref_.getData()); }
	inline void getPointFromDepth(int depth_x, int depth_y, float &point_x, float &point_y, float &point_z) const { openni::CoordinateConverter::convertDepthToWorld(depth_stream_, depth_x, depth_y, getDepthDataPtr()[depth_x + getWidth() * depth_y], &point_x, &point_y, &point_z); }

	// Processing
	int open(const char* uri = 0);
	inline int open(const std::string &uri) { return open(uri.c_str()); }
	int close();
	int acquire();
	inline int update() { return acquire(); }
	int startRecording(const char *uri);
	inline int startRecording(const std::string &uri) { return startRecording(uri.c_str()); }
	int stopRecording();
	virtual int copyDepthDataTo(unsigned short* depth_data) const = 0;
	virtual int copyDepthDataTo(unsigned char* depth_data) const = 0;
	virtual int copyInfraredDataTo(unsigned short* infrared_data) const = 0;
	virtual int copyInfraredDataTo(unsigned char* infrared_data) const = 0;
	virtual int copyColorDataTo(unsigned short* color_data) const = 0;
	virtual int copyColorDataTo(unsigned char* color_data) const = 0;
};



#endif	// Enf of #ifndef OPENNI2_GRABBER_VERSION