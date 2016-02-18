#pragma once
#include <librealsense/rs.hpp>
#include <iostream>
#include <string>
#include <stdio.h>
#include <opencv2\opencv.hpp>
#include "device_interface.h"
class RealSenseInterface : public DeviceInterface
{
public:
	RealSenseInterface();
	RealSenseInterface(const size_t, const size_t);
	~RealSenseInterface();
	virtual void capture_frame(cv::Mat &, cv::Mat &, cv::Mat &, bool);
	virtual void start_device(); //start the kinect device using LibFreenect2
	virtual void stop_device(); //stop the device
	virtual void clear_frame_buffer(); //clear the frame buffer
	virtual void process_frame_buffer(std::string filename); //process the frame buffer and save images
	virtual void init_ar(const float marker_size, const string board_fn);
	virtual size_t num_frames();

private:
	//get individual given a RealSense stream
	void get_frame(cv::Mat& frame, rs::stream stream, int *timestamp);

	rs::context ctx;
	rs::device *dev;

};

