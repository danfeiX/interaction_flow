//
// Created by danfei on 1/22/16.
//
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include "kinect_interface.h"
#include "flow_utils.h"
using namespace std;

KinectInterface::KinectInterface(const size_t im_w, const size_t im_h):
backend(Processor::cl)
{
	im_width = im_w;
	im_height = im_h;
}


void KinectInterface::start_device() {
	//! [context]
	dev = nullptr;
	registration = nullptr;
	libfreenect2::PacketPipeline *pipeline = nullptr;
	//! [context]

	//! [discovery]
	if (freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
	}

	string serial = freenect2.getDefaultDeviceSerialNumber();

	std::cout << "SERIAL: " << serial << std::endl;
	//! [discovery]

	int depthProcessor = backend;

	if (depthProcessor == Processor::cpu)
	{
		if (!pipeline)
			//! [pipeline]
			pipeline = new libfreenect2::CpuPacketPipeline();
		//! [pipeline]
	}
	else if (depthProcessor == Processor::gl) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
		if (!pipeline)
			pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
		std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
	}
	else if (depthProcessor == Processor::cl) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
		if (!pipeline) {
			pipeline = new libfreenect2::OpenCLPacketPipeline();
		}
		else {
			std::cout << "Using OpenCL backend!" << std::endl;

		}
#else
		std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
	}

	if (pipeline)
	{
		//! [open]
		dev = freenect2.openDevice(serial, pipeline);
		//! [open]
	}
	else {
		dev = freenect2.openDevice(serial);
	}

	if (!dev)
	{
		std::cout << "failure opening device!" << std::endl;
	}

	//! [listeners]
	listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
		libfreenect2::Frame::Depth);
	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);

	//frame buffers
	undistorted = new libfreenect2::Frame(512, 424, 4);
	registered = new libfreenect2::Frame(512, 424, 4);
	depth2rgb = new libfreenect2::Frame(1920, 1080 + 2, 4);

    cout <<"starting device\n" <<endl;
    dev->start();
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

void KinectInterface::stop_device() {
    dev->stop();
}

//capture the latest frame from Kinect
//buffer: choice of:
//1. buffer raw frames and post process the buffer later
//2. buffer registered frames
//3. nothing
void KinectInterface::capture_frame(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz, bool buffer) {
    libfreenect2::FrameMap frame;
    listener->waitForNewFrame(frame);
	libfreenect2::Frame *rgb_frame = frame[libfreenect2::Frame::Color];
	libfreenect2::Frame *depth_frame = frame[libfreenect2::Frame::Depth];

	//buffer and real time registration
	register_depth(rgb_frame, depth_frame, rgb, depth);
	depth_to_xyz(depth, xyz);
	//threshPosition(depth, xyz, cv::Point3f(BOUND_MIN_X, BOUND_MIN_Y, BOUND_MIN_Z), 
	//						   cv::Point3f(BOUND_MAX_X, BOUND_MAX_Y, BOUND_MAX_Z));
	if (buffer) {
		Mat rgb_, depth_, xyz_;
		rgb.copyTo(rgb_);
		depth.copyTo(depth_);
		xyz.copyTo(xyz_);
		rgb_buffer.push_back(rgb_);
		depth_buffer.push_back(depth_);
		xyz_buffer.push_back(xyz_);
	}
	//ar.detect_board(intensity);

}

KinectInterface::~KinectInterface() {
    dev->close();
    if (registration)
        delete registration;
    delete listener;
    delete undistorted;
    delete registered;
    delete depth2rgb;
}

void KinectInterface::init_ar(const float marker_size, const string board_fn) {
	libfreenect2::Freenect2Device::IrCameraParams param = dev->getIrCameraParams();

	float cat_data[9] = {param.fx, 0, param.cx, 0, param.fy, param.cy, 0, 0, 1};
	float dist_data[4] = {param.k1, param.k2, param.p1, param.p2};

	cv::Mat cam_mat = cv::Mat(3, 3, CV_32F, cat_data);
	cv::Mat distortion = cv::Mat(4, 1, CV_32F, dist_data);

	ar.init_detector(cam_mat, distortion, cv::Size(im_width, im_height), marker_size, board_fn);
}


void KinectInterface::register_depth(const Frame* rgb, const Frame* depth, cv::Mat & out_rgb, cv::Mat & out_depth) {
    cv::Mat depth_mat, depth2rgb_mat;
    //original rgb
	cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(out_rgb);
    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depth_mat);

    registration->apply(rgb, depth, undistorted, registered, true, depth2rgb);
    cv::Mat(depth2rgb->height, depth2rgb->width, CV_32FC1, depth2rgb->data).copyTo(depth2rgb_mat);

	cv::resize(out_rgb(cv::Rect(240, 0, 1440, 1080)), out_rgb, cv::Size(im_width, im_height));
    cv::resize(depth2rgb_mat(cv::Rect(240, 0, 1440, 1080)), out_depth, cv::Size(im_width, im_height));
    out_depth = out_depth*0.001f; //mm - > m
}


void KinectInterface::depth_to_xyz(const cv::Mat& rectified_depth, cv::Mat& outXYZ) {
	const float cx = dev->getIrCameraParams().cx;
	const float cy = dev->getIrCameraParams().cy;
	const float fx = 1/dev->getIrCameraParams().fx;
	const float fy = 1/dev->getIrCameraParams().fy;

	const float bad_val = std::numeric_limits<float>::quiet_NaN();
	const cv::Point3f bad_pos(bad_val, bad_val, bad_val);
	outXYZ = cv::Mat(rectified_depth.rows, rectified_depth.cols, CV_32FC3);

	for (int r = 0; r < rectified_depth.rows; ++r) {
		for (int c = 0; c < rectified_depth.cols; ++c) {
			float depth_val = rectified_depth.at<float>(r, c);
			if (isnan(depth_val) || depth_val <= 0.001)
			{
				//depth value is not valid
				outXYZ.at<cv::Point3f>(r, c) = bad_pos;
			}
			else {
				float x = (c + 0.5 - cx) * fx * depth_val;
				float y = (r + 0.5 - cy) * fy * depth_val;
				float z = depth_val;
				outXYZ.at<cv::Point3f>(r, c) = cv::Point3f(x, y, z);
			}
		}
	}
}

//1. Converts the rgb and depth frame buffer to cv::Mat that is of
//the input format of PD-flow.
//2. Writes the cv::Mat to a binary file
void KinectInterface::process_frame_buffer(std::string filename) {
	cout << "processing buffer..." << endl;
	ofstream output_file(filename.c_str(), ios::binary);
	size_t num_frame = num_frames();
	output_file.write((char *)&num_frame, sizeof(size_t));

	for (int i = 0; i < num_frames(); ++i) {
		Mat intensity;
		writeMatBinary(output_file, rgb_buffer[i]);
		writeMatBinary(output_file, depth_buffer[i]);
		writeMatBinary(output_file, xyz_buffer[i]);

		cv::imshow("depth2rgb", depth_buffer[i]);
		cv::imshow("intensity", xyz_buffer[i]);
		int key = cv::waitKey(1);
	}

}

size_t KinectInterface::num_frames()
{
	assert(rgb_buffer.size() == depth_buffer.size() == xyz_buffer.size());
	return rgb_buffer.size();
}

void KinectInterface::clear_frame_buffer() {
}




#if 0
void KinectInterface::save_buffer(std::string filename) {
	ofstream output_file(filename.c_str(), ios::binary);
	size_t num_frame = rgb_frame_buffer.size();
	output_file.write((char *)&num_frame, sizeof(size_t));
	libfreenect2::Frame *frame;
	for (int i = 0; i < rgb_frame_buffer.size(); ++i) {
		frame = rgb_frame_buffer[i];
		output_file.write((char *)frame, sizeof(libfreenect2::Frame));
		size_t data_size = frame->width * frame->height * frame->bytes_per_pixel;
		output_file.write((char *)frame->data, data_size);

		frame = depth_frame_buffer[i];
		output_file.write((char *)frame, sizeof(libfreenect2::Frame));
		data_size = frame->width * frame->height * frame->bytes_per_pixel;
		output_file.write((char *)frame->data, data_size);
	}
	printf("Saved %i frames\n", rgb_frame_buffer.size());
}

void KinectInterface::load_buffer(std::string filename) {
	ifstream input_file(filename.c_str(), ios::binary);
	size_t num_frame;
	input_file.read((char *)&num_frame, sizeof(size_t));
	unsigned char dummy;
	libfreenect2::Frame *frame;
	for (int i = 0; i < num_frame; ++i) {
		frame = new libfreenect2::Frame(0, 0, 0, &dummy);
		input_file.read((char *)frame, sizeof(libfreenect2::Frame));
		size_t data_size = frame->width * frame->height * frame->bytes_per_pixel;
		frame->data = new unsigned char[data_size];
		input_file.read((char *)frame->data, data_size);
		rgb_frame_buffer.push_back(frame);

		frame = new libfreenect2::Frame(0, 0, 0, &dummy);
		input_file.read((char *)frame, sizeof(libfreenect2::Frame));
		data_size = frame->width * frame->height * frame->bytes_per_pixel;
		frame->data = new unsigned char[data_size];

		input_file.read((char *)frame->data, data_size);
		depth_frame_buffer.push_back(frame);
	}
	printf("Read %i frames\n", rgb_frame_buffer.size());

}
#endif // 0
