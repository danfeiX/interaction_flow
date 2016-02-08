//
// Created by danfei on 1/22/16.
//
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include "kinect_interface_win.h"
using namespace std;

// Intermediate Buffers

#define CHECKERROR(HR) if (!SUCCEEDED(HR)) printf("Error at %s in %d" , __FILE__, __LINE__);


const int width = 512;
const int height = 424;
const int colorwidth = 1920;
const int colorheight = 1080;

unsigned char rgbimage[colorwidth*colorheight * 4];    // Stores RGB color image
ColorSpacePoint depth2rgb[width*height];             // Maps depth pixels to rgb pixels
CameraSpacePoint depth2xyz[width*height];			 // Maps depth pixels to 3d coordinates
UINT16 depth_data[height * width];


KinectInterface::KinectInterface() {

}


bool KinectInterface::start_device() {
	HRESULT hr;
	kinectSensor = nullptr;
	hr = GetDefaultKinectSensor(&kinectSensor);

	// initialize Kinect Sensor
	if (FAILED(hr) || !kinectSensor) {
		std::cout << "ERROR hr=" << hr << "; sensor=" << kinectSensor << std::endl;
		return false;
	}
	CHECKERROR(kinectSensor->Open())
	kinectSensor->get_CoordinateMapper(&mapper);

	// initialize depth frame reader
	//CHECKERROR(kinectSensor->OpenMultiSourceFrameReader(
	//	FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
	//	&reader));
	IDepthFrameSource* depthFrameSource = nullptr;
	CHECKERROR(kinectSensor->get_DepthFrameSource(&depthFrameSource));
	CHECKERROR(depthFrameSource->OpenReader(&depthFrameReader));
	if (depthFrameSource) depthFrameSource->Release();

	IColorFrameSource* colorFrameSource = nullptr;
	CHECKERROR(kinectSensor->get_ColorFrameSource(&colorFrameSource));
	CHECKERROR(colorFrameSource->OpenReader(&colorFrameReader));
	if (depthFrameSource) colorFrameSource->Release();

	return true;
}

void KinectInterface::stop_device() {
}

bool KinectInterface::capture_frame() {
	IMultiSourceFrame *frame = nullptr;
	cv::Mat intensity, depth, pos;
	if (getFrameData(frame, intensity, depth, pos)) {
		cv::imshow("intensity", intensity);
		cv::Mat depth_f;
		depth.convertTo(depth_f, CV_32FC1);
		cv::imshow("depth", depth_f / 6500);
		if (frame) frame->Release();
		return true;
	}
	return false;
}


bool KinectInterface::getFrameData(IMultiSourceFrame* frame, cv::Mat& intensity_mat, cv::Mat& depth_mat, cv::Mat& pos_mat) {
	//Obtain depth frame
	IDepthFrame* depthframe = nullptr;
	if (FAILED(depthFrameReader->AcquireLatestFrame(&depthframe))) return false;
	if (!depthframe) return false;
	// Get data from frame
	unsigned int sz;
	unsigned short* buf;
	if (FAILED(depthframe->AccessUnderlyingBuffer(&sz, &buf))) return false;
	//get depth -> xyz mapping
	if (FAILED(mapper->MapDepthFrameToCameraSpace(width*height, buf, width*height, depth2xyz))) return false;
	//get depth -> rgb image mapping 
	if (FAILED(mapper->MapDepthFrameToColorSpace(width*height, buf, width*height, depth2rgb))) return false;
	//save depth
	if (FAILED(depthframe->CopyFrameDataToArray(height * width, depth_data)));

	if (depthframe) depthframe->Release();


	//Obtain RGB frame
	IColorFrame* colorframe;
	if (FAILED(colorFrameReader->AcquireLatestFrame(&colorframe))) return false;
	if (!colorframe) return false;

	// Get data from frame
	if (FAILED(colorframe->CopyConvertedFrameDataToArray(colorwidth*colorheight * 4, rgbimage, ColorImageFormat_Rgba))) return false;


	cv::Mat tmp_depth = cv::Mat::zeros(colorheight, colorwidth, CV_16UC1);
	cv::Mat tmp_pos = cv::Mat::zeros(colorheight, colorwidth, CV_32FC3);
	cv::Mat depth_org(height, width, CV_16UC1, depth_data);
	cv::Mat tmp_rgb(colorheight, colorwidth, CV_8UC4, rgbimage);

	// Write color array for vertices
	for (int i = 0; i < width*height; i++) {
		ColorSpacePoint p = depth2rgb[i];
		int iY = (int)(p.Y + 0.5);
		int iX = (int)(p.X + 0.5);
		if (iX >= 0 && iY >= 0 && iX < colorwidth && iY < colorheight) {
			// Check if color pixel coordinates are in bounds
			tmp_depth.at<unsigned short>(iY, iX) = depth_data[i];
			//tmp_pos.at<float>(iY, iX, 0) = depth2xyz[i].X;
			//tmp_pos.at<float>(iY, iX, 1) = depth2xyz[i].Y;
			//tmp_pos.at<float>(iY, iX, 2) = depth2xyz[i].Z;
		}
	}

	if (colorframe) colorframe->Release();

	cv::resize(tmp_rgb(cv::Rect(240, 0, 1440, 1080)), intensity_mat, cv::Size(640, 480));
	cv::resize(tmp_depth(cv::Rect(240, 0, 1440, 1080)), depth_mat, cv::Size(640, 480));
	cv::resize(tmp_pos(cv::Rect(240, 0, 1440, 1080)), pos_mat, cv::Size(640, 480));
	cv::cvtColor(intensity_mat, intensity_mat, CV_RGBA2GRAY);
	return true;
}

void KinectInterface::save_buffer(std::string filename) {
	ofstream output_file(filename.c_str(), ios::binary);

	size_t num_frame = rgb_buffer.size();
    output_file.write((char *)&num_frame, sizeof(size_t));
    for (int i = 0 ; i < rgb_buffer.size(); ++i) {
		writeMatBinary(output_file, rgb_buffer[i]);
		writeMatBinary(output_file, depth_buffer[i]);
    }
    printf("Saved %i frames\n", rgb_buffer.size());
}

void KinectInterface::load_buffer(std::string filename) {
    ifstream input_file(filename.c_str(), ios::binary);
    size_t num_frame;
    input_file.read((char *)&num_frame, sizeof(size_t));
    for (int i = 0 ; i < num_frame; ++i) {
		cv::Mat rgb, depth;
		readMatBinary(input_file, rgb);
		readMatBinary(input_file, depth);
		rgb_buffer.push_back(rgb);
        depth_buffer.push_back(depth);
    }
    printf("Read %i frames\n", rgb_buffer.size());

}

void KinectInterface::clear_buffer() {
}

KinectInterface::~KinectInterface() {

	// de-initialize Kinect Sensor
	CHECKERROR(kinectSensor->Close());
	if (kinectSensor)
		kinectSensor->Release();
}


void KinectInterface::process_frame_PDFlow(size_t frameID, cv::Mat& out_intensity, cv::Mat& out_depth) {
    process_frame_PDFlow(rgb_buffer[frameID], depth_buffer[frameID], out_intensity, out_depth);
}

void KinectInterface::process_frame_PDFlow(cv::Mat & rgb_mat, cv::Mat & depth_mat, cv::Mat & out_intensity, cv::Mat & out_depth) {

    cv::resize(rgb_mat(cv::Rect(240, 0, 1440, 1080)), out_intensity, cv::Size(640, 480));
	cv::resize(depth_mat(cv::Rect(240, 0, 1440, 1080)), out_depth, cv::Size(640, 480));
    cv::cvtColor(out_intensity, out_intensity, CV_BGR2GRAY);
    out_depth = out_depth*0.001f;
}

void KinectInterface::process_buffer() {
    cv::Mat rgb_mat, depth_mat, depth2rgb_mat;
}


//! Write cv::Mat as binary
/*!
\param[out] ofs output file stream
\param[in] out_mat mat to save
*/
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
bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat)
{
	if (!ifs.is_open()){
		return false;
	}

	int rows, cols, type;
	ifs.read((char*)(&rows), sizeof(int));
	if (rows == 0){
		return true;
	}
	ifs.read((char*)(&cols), sizeof(int));
	ifs.read((char*)(&type), sizeof(int));

	in_mat.release();
	in_mat.create(rows, cols, type);
	ifs.read((char*)(in_mat.data), in_mat.elemSize() * in_mat.total());

	return true;
}
