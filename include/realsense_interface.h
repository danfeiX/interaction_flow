#pragma once
#include <librealsense/rs.hpp>
#include <iostream>
#include <string>
#include <stdio.h>
#include "device_interface.h"
class RealSenseInterface : public DeviceInterface
{
public:
	RealSenseInterface();
	RealSenseInterface(size_t w, size_t h, bool buffer, size_t device_id, rs::context *ctx);
	~RealSenseInterface();
	virtual void capture_frame();
	virtual void start_device(); //start the kinect device using LibFreenect2
	virtual void stop_device(); //stop the device
	virtual void clear_frame_buffer(); //clear the frame buffer
	virtual void init_ar(const float marker_size, const std::string board_fn);
	virtual size_t num_frames();

private:
	//get individual given a RealSense stream
	void get_frame(cv::Mat& frame, const rs::stream stream, int *timestamp);
	void depth_to_xyz(const cv::Mat& rectified_depth, cv::Mat& outXYZ);
	void color_to_depth(const cv::Mat& color, cv::Mat& outDepth);

	rs::context *m_ctx;
	rs::device *m_dev;

	rs::intrinsics m_intrinsics;
};

