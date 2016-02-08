//
// Created by danfei on 1/22/16.
//
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include "kinect_interface.h"
using namespace std;

KinectInterface::KinectInterface() {
}


void KinectInterface::start_device(Processor backend) {
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

void KinectInterface::capture_frame() {
    libfreenect2::FrameMap frame;
    listener->waitForNewFrame(frame);
    rgb_frame_buffer.push_back(frame[libfreenect2::Frame::Color]);
	depth_frame_buffer.push_back(frame[libfreenect2::Frame::Depth]);

	cv::Mat rgb_mat;
	cv::Mat(rgb_frame_buffer.back()->height, 
			rgb_frame_buffer.back()->width, 
			CV_8UC4, 
			rgb_frame_buffer.back()->data).copyTo(rgb_mat);

	cv::resize(rgb_mat, rgb_mat, cv::Size(640, 360));
	cv::imshow("rgb", rgb_mat);
}


void KinectInterface::save_buffer(std::string filename) {
    ofstream output_file(filename.c_str(), ios::binary);
    size_t num_frame = rgb_frame_buffer.size();
    output_file.write((char *)&num_frame, sizeof(size_t));
    libfreenect2::Frame *frame;
	for (int i = 0; i < rgb_frame_buffer.size(); ++i) {
		frame = rgb_frame_buffer[i];
        output_file.write((char *) frame, sizeof(libfreenect2::Frame));
        size_t data_size = frame->width * frame->height * frame->bytes_per_pixel;
        output_file.write((char *) frame->data, data_size);

		frame = depth_frame_buffer[i];
        output_file.write((char *) frame, sizeof(libfreenect2::Frame));
        data_size = frame->width * frame->height * frame->bytes_per_pixel;
        output_file.write((char *) frame->data, data_size);
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

void KinectInterface::clear_buffer() {
	for (int i = 0; i < rgb_frame_buffer.size(); ++i) {
		delete rgb_frame_buffer[i];
        delete depth_frame_buffer[i];
    }
}

KinectInterface::~KinectInterface() {
    stop_device();
    dev->close();
    if (registration)
        delete registration;
    delete listener;
    delete undistorted;
    delete registered;
    delete depth2rgb;
}


void KinectInterface::process_frame_PDFlow(size_t frameID, cv::Mat& out_intensity, cv::Mat& out_depth, cv::Mat & out_rgb) {
	process_frame_PDFlow(rgb_frame_buffer[frameID], depth_frame_buffer[frameID], out_intensity, out_depth, out_rgb);
}


void KinectInterface::process_frame_PDFlow(const Frame* rgb, const Frame* depth, cv::Mat & out_intensity, cv::Mat & out_depth, cv::Mat & out_rgb) {
    cv::Mat depth_mat, depth2rgb_mat;
    //original rgb
	cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(out_rgb);
    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depth_mat);

    registration->apply(rgb, depth, undistorted, registered, true, depth2rgb);
    cv::Mat(depth2rgb->height, depth2rgb->width, CV_32FC1, depth2rgb->data).copyTo(depth2rgb_mat);

	cv::resize(out_rgb(cv::Rect(240, 0, 1440, 1080)), out_intensity, cv::Size(640, 480));
    cv::resize(depth2rgb_mat(cv::Rect(240, 0, 1440, 1080)), out_depth, cv::Size(640, 480));
    cv::cvtColor(out_intensity, out_intensity, CV_BGR2GRAY);
    out_depth = out_depth*0.001f; //mm - > m
}


void KinectInterface::process_frame_XYZ(const cv::Mat& rectified_depth, cv::Mat& outXYZ) {
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

void KinectInterface::postprocess_buffer(std::string filename) {
	ofstream output_file(filename.c_str(), ios::binary);
	size_t num_frame = num_frames();
	output_file.write((char *)&num_frame, sizeof(size_t));

    cv::Mat rgb_mat, depth_mat, depth2rgb_mat;

    for (int i = 0; i < num_frames(); ++i) {
		libfreenect2::Frame *rgb_frame = rgb_frame_buffer[i];
		libfreenect2::Frame *depth_frame = depth_frame_buffer[i];

        cv::Mat intensity, depth, rgb, xyz;

        process_frame_PDFlow(rgb_frame, depth_frame, intensity, depth, rgb);
		process_frame_XYZ(depth, xyz);
		writeMatBinary(output_file, intensity);
		writeMatBinary(output_file, rgb);
		writeMatBinary(output_file, depth);
		writeMatBinary(output_file, xyz);

		cv::imshow("depth2rgb", depth);
		cv::imshow("rgb", rgb);
        int key = cv::waitKey(1);
    }
}


bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
{
	if (!ofs.is_open()){
		return false;
	}
	if (out_mat.empty()){
		int s = 0;
		ofs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = out_mat.type();
	ofs.write((const char*)(&out_mat.rows), sizeof(int));
	ofs.write((const char*)(&out_mat.cols), sizeof(int));
	ofs.write((const char*)(&type), sizeof(int));
	ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

	return true;
}


//! Read cv::Mat from binary
/*!
\param[in] ifs input file stream
\param[out] in_mat mat to load
*/
