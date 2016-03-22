#pragma once
#ifndef DEVICE_INTERFACE_H
#define DEVICE_INTERFACE_H
#include <string>
#include <opencv2\opencv.hpp>
#include <mutex> //std::thread
#include <thread>
#include "ARScene.h"
#include "flow_utils.h"

#define MARKER_SIZE 0.087 //physical size of the AR marker



struct RGBDFrame
{
	int timestamp;
	cv::Mat rgb;
	cv::Mat depth;
	float cx, cy, fx, fy; //depth -> xyz
	RGBDFrame(const cv::Mat & irgb, const cv::Mat & idepth, const int ts)
	{
		irgb.copyTo(rgb);
		idepth.copyTo(depth);
		timestamp = ts;
	}
};


class DeviceInterface
{
	typedef lock_guard<std::mutex> scope_guard;
public:
	DeviceInterface(size_t w, size_t h, size_t device_id);
	DeviceInterface();
	~DeviceInterface();

	virtual void capture_frame() = 0; //capture the latest rgb and depth frames from the device
	virtual void start_device() = 0; //start the camera device
	virtual void stop_device() = 0; //stop the camera device
	virtual void init_ar(const float marker_size, const std::string board_fn) = 0; //initialize a VR calibration tool

	void start_all(); //a script that start the device, load calibration matrix, and spawn the capturing thread 
	void stop_all(); //a script that stop the capturing thread, stop the device, and clear the buffer

	void spawn_capture_thread(); //spawn a separate thread for capturing
	void stop_capture_thread(); // stop the thread for capturing

	template<typename PT, typename C>
	bool get_latest_point_cloud(vector<PT> & pointcloud, vector<C> & color); //access the latest point cloud
	bool get_latest_frame(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz); //access the latest frame

	void save_raw_frames(const std::string filename); //save raw frames
	void clear_frame_buffer(); //clear the frame buffer
	size_t num_frames(); //number of frames in the current buffer

	void calibrateCameraPose(const size_t num_frames); //calibrate pose of the camera
	void loadCalibrationFile(const std::string filename); //load camera pose calibration matrix
	void depth_to_xyz(const cv::Mat & depth, cv::Mat & xyz);

	void lock_thread() { m_lock.lock(); }
	void unlock_thread() { m_lock.unlock(); }

	void toggle_buffer() { scope_guard sg(m_lock);  do_buffer = 1 - do_buffer; }

	template<typename PT, typename C>
	void rgbd_to_point_cloud(const cv::Mat & xyz, const cv::Mat & rgb, vector<PT> & pointcloud, vector<C> & color);

	cv::Mat m_cam_to_world; //camera matrix, from camera space to world
	size_t m_device_id; //ID of the current device

	//frame buffers
	std::vector<RGBDFrame> rgbd_buffer;
protected:
	//image dimensions
	size_t im_width;
	size_t im_height;

	//Update buffer and latest frame pointer, needs to lock thread if multi-threading
	void update(const cv::Mat & rgb, const cv::Mat & depth, const int timestamp);
	void capture_callback();

	//thread and its lock for multi-threading capturing
	std::mutex m_lock;
	std::thread m_thread;

	//mutex-protected container of the newest frame
	cv::Mat curr_rgb; 
	cv::Mat curr_depth;
	// cv::Mat * curr_xyz;

	bool do_buffer; //if save captured frames
	bool m_thread_running; //thread flag

	ARScene ar; //ar module used for camera pose calibration
	float cx, cy, fx, fy; // camera intrinsic parameters;
};


//get point cloud in the world frame from the latest frame
template<typename PT, typename C>
bool DeviceInterface::get_latest_point_cloud(vector<PT> & point_cloud, vector<C> & color)
{
	cv::Mat rgb, depth, xyz;
	if (!get_latest_frame(rgb, depth, xyz))
		return false;
	rgbd_to_point_cloud<PT, C>(xyz, rgb, point_cloud, color);
	return true;
}

template<typename PT, typename C>
void DeviceInterface::rgbd_to_point_cloud(const cv::Mat & xyz, const cv::Mat & rgb, vector<PT> & point_cloud, vector<C> & color)
{
	Mat t_pts;
	transformPointCloud(t_pts, xyz, m_cam_to_world); //transform XYZ from camera frame to world frame
	for (int i = 0; i < t_pts.cols; ++i)
	{
		PT pt = PT(t_pts.at<float>(0, i), t_pts.at<float>(1, i), t_pts.at<float>(2, i));
		float scale = t_pts.at<float>(3, i);
		if (!isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z)) {
			point_cloud.push_back(pt / scale);
			cv::Vec3b pc = rgb.at<cv::Vec3b>(i);
			color.push_back(C((float)pc[2] / 255, (float)pc[1] / 255, (float)pc[0] / 255, 1.0f));
		}
	}
}



#endif