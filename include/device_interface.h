#pragma once
#include <string>
#include <opencv2\opencv.hpp>
#include <mutex> //std::thread
#include <thread>
#include "ARScene.h"

#define MARKER_SIZE 0.087

class DeviceInterface
{
public:
	DeviceInterface(size_t w, size_t h, bool buffer, size_t device_id);
	DeviceInterface();
	~DeviceInterface();
	virtual void capture_frame() = 0;
	virtual void start_device() = 0; //start the kinect device using LibFreenect2
	virtual void stop_device() = 0; //stop the device
	virtual void clear_frame_buffer() = 0; //clear the frame buffer
	virtual void init_ar(const float marker_size, const std::string board_fn) = 0;
	virtual size_t num_frames() = 0;

	void start_all() 
	{
		start_device();
		spawn_capture_thread();
		init_ar(MARKER_SIZE, "D:/workspace/interaction_flow/ar/board.conf");
	}

	void stop_all()
	{
		stop_capture();
		stop_device();
		clear_frame_buffer();
	}


	void spawn_capture_thread();
	void stop_capture();
	bool get_latest_frame(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz);
	void process_frame_buffer(const std::string filename); //process the frame buffer and save images
	void calibrateCameraPose(const size_t num_frames); //calibrate pose of the camera
	void loadCalibrationFile(const std::string filename);
	cv::Mat m_cam_to_world;
	size_t m_device_id;

protected:
	//Update buffer and latest frame pointer, needs to lock thread if multi-threading
	void update(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz);
	void capture_callback();


	std::mutex m_lock;
	std::thread m_thread;

	size_t im_width;
	size_t im_height;

	std::vector<cv::Mat> rgb_buffer;
	std::vector<cv::Mat> depth_buffer;
	std::vector<cv::Mat> xyz_buffer;

	//mutex-protected pointer to the newest frame

	cv::Mat * curr_rgb;
	cv::Mat * curr_xyz;
	cv::Mat * curr_depth;

	bool do_buffer;
	bool thread_running;


	//ar
	ARScene ar;
};