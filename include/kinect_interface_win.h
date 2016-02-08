//
// Created by danfei on 1/22/16.
//

#ifndef INTERACTION_FLOW_KINECT_INTERFACE_WIN_H
#define INTERACTION_FLOW_KINECT_INTERFACE_WIN_H

#include <Windows.h>
#include <Kinect.h>
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <io.h>


bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);


class KinectInterface {
public:
    KinectInterface();
    bool capture_frame();
    ~KinectInterface();
    bool start_device();
    void stop_device();
    void save_buffer(std::string filename);
    void load_buffer(std::string filename);
    void clear_buffer();
    void process_buffer();
    void process_frame_PDFlow(cv::Mat&, cv::Mat&, cv::Mat &, cv::Mat &);
    void process_frame_PDFlow(size_t frameID, cv::Mat&, cv::Mat&);
    size_t num_frames() {return rgb_buffer.size();}


private:

	std::vector<cv::Mat> rgb_buffer;
	std::vector<cv::Mat> depth_buffer;
	IKinectSensor* kinectSensor;
	IMultiSourceFrameReader* reader;
	ICoordinateMapper* mapper;

	IDepthFrameReader* depthFrameReader;
	IColorFrameReader* colorFrameReader; 
	bool getFrameData(IMultiSourceFrame*, cv::Mat&, cv::Mat&, cv::Mat&);



};
#endif //INTERACTION_FLOW_KINECT_INTERFACE_WIN_H
