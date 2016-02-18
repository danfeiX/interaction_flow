#pragma once
#include <string>
#include <opencv2\opencv.hpp>
using namespace std;


class DeviceInterface
{
public:
	~DeviceInterface();
	virtual void capture_frame(cv::Mat &, cv::Mat &, cv::Mat &, bool buffer) = 0;
	virtual void start_device() = 0; //start the kinect device using LibFreenect2
	virtual void stop_device() = 0; //stop the device
	virtual void clear_frame_buffer() = 0; //clear the frame buffer
	virtual void process_frame_buffer(std::string filename) = 0; //process the frame buffer and save images
	virtual void init_ar(const float marker_size, const string board_fn) = 0;
	virtual size_t num_frames() = 0;

protected:
	size_t im_width;
	size_t im_height;

	vector<cv::Mat> rgb_buffer;
	vector<cv::Mat> depth_buffer;
	vector<cv::Mat> xyz_buffer;
};