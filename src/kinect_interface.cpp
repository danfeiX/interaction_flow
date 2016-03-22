//
// Created by danfei on 1/22/16.
//
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include "kinect_interface.h"
#include "flow_utils.h"
using namespace std;

KinectInterface::KinectInterface(size_t w, size_t h, size_t device_id) :
backend(Processor::cl),
DeviceInterface(w, h, device_id)
{
}


KinectInterface::KinectInterface():
backend(Processor::cl)
{
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
	cx = dev->getIrCameraParams().cx;
	cy = dev->getIrCameraParams().cy;
	fx = dev->getIrCameraParams().fx;
	fy = dev->getIrCameraParams().fy;
}


void KinectInterface::stop_device() {
    dev->stop();
}


void KinectInterface::capture_frame() {
	Mat xyz, rgb, depth;
    libfreenect2::FrameMap frame;
    listener->waitForNewFrame(frame);
	libfreenect2::Frame *rgb_frame = frame[libfreenect2::Frame::Color];
	libfreenect2::Frame *depth_frame = frame[libfreenect2::Frame::Depth];

	//buffer and real time registration
	register_depth(rgb_frame, depth_frame, rgb, depth);
	update(rgb, depth, 0);
}


KinectInterface::~KinectInterface() 
{
    dev->close();
    if (registration)
        delete registration;
    delete listener;
    delete undistorted;
    delete registered;
    delete depth2rgb;
}


void KinectInterface::init_ar(const float marker_size /*in meter*/, const string board_fn) {
	libfreenect2::Freenect2Device::ColorCameraParams param = dev->getColorCameraParams();

	float cat_data[9] = {param.fx, 0, param.cx, 0, param.fy, param.cy, 0, 0, 1};
	float dist_data[4] = {0, 0, 0, 0};

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
