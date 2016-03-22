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
	RealSenseInterface(size_t w, size_t h, size_t device_id, rs::context *ctx);
	~RealSenseInterface();

	virtual void capture_frame();
	virtual void start_device(); //start the kinect device using LibRealSense
	virtual void stop_device(); //stop the device
	virtual void init_ar(const float marker_size, const std::string board_fn);

private:
	//get individual given a RealSense stream
	void get_frame(cv::Mat& frame, const rs::stream stream, int *timestamp);
	void color_to_depth(const cv::Mat& color, cv::Mat& outDepth);

	rs::context *m_ctx;
	rs::device *m_dev;

	rs::intrinsics m_intrinsics;
};

