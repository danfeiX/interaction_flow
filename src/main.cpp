#include <iostream>
#include <iomanip>

#include "kinect_interface.h"
#include "realsense_interface.h"
#include "scene_flow.h"
#include "flow_utils.h"


using namespace std;
using namespace cv;

//high resolution flow (640x480)
#define HIGH_RES 0 
//load raw data (rgb, intensity, depth, xyz) from a database
void loadDB(const std::string filename, std::vector<cv::Mat*>& intensity_v, std::vector<cv::Mat*>& depth_v, std::vector<cv::Mat*>& xyz_v)
{
	ifstream input_file(filename.c_str(), ios::binary);
	size_t num_frame;
	input_file.read((char *)&num_frame, sizeof(size_t));
	cout << "num frame: " << num_frame << endl;
	for (int i = 0; i < num_frame; ++i) {
		Mat * rgb = new Mat();
		Mat * intensity = new Mat();
		Mat * depth = new Mat();
		Mat * xyz = new Mat();
		readMatBinary(input_file, *intensity);
		//readMatBinary(input_file, *rgb);
		readMatBinary(input_file, *depth);
		readMatBinary(input_file, *xyz);
		threshPosition(*depth, *xyz, cv::Point3f(BOUND_MIN_X, BOUND_MIN_Y, BOUND_MIN_Z), cv::Point3f(BOUND_MAX_X, BOUND_MAX_Y, BOUND_MAX_Z));

		intensity_v.push_back(intensity);
		depth_v.push_back(depth);
		xyz_v.push_back(xyz);
	}
}

//free allocated DB space
void freeDB(std::vector<cv::Mat*>& intensity_v, std::vector<cv::Mat*>& depth_v, std::vector<cv::Mat*>& xyz_v)
{
	for (int i = 0; i < intensity_v.size(); ++i) {
		delete intensity_v[i];
		delete depth_v[i];
		delete xyz_v[i];
	}
}

void displayView(cv::Mat& display, cv::Mat & intensity, cv::Mat &depth, cv::Mat& xyz, cv::Mat &flow_color)
{
	Mat i1_, d1_, xyz_;
	Mat disp = Mat::zeros(display.rows, display.cols, CV_32FC3);
	cv::cvtColor(intensity, i1_, COLOR_GRAY2RGB);
	i1_.convertTo(i1_, CV_32FC3);
	i1_ = i1_ / 255;
	cv::cvtColor(depth, d1_, COLOR_GRAY2RGB);
	i1_.copyTo(disp(cv::Rect(0, 0, 640, 480)));
	d1_.copyTo(disp(cv::Rect(640, 0, 640, 480)));
	xyz.copyTo(disp(cv::Rect(640, 480, 640, 480)));
	flow_color.copyTo(disp(cv::Rect(0, 480, 640, 480))); \
		disp = disp * 255;
	disp.convertTo(display, CV_8UC3);
}

//Write out position and flow for each frame as a binary file of arrays
void writeOut(std::ofstream &ofs, const cv::Mat& xyz, cv::Mat& flow)
{
	int frame_size = xyz.total();
	ofs.write((const char*)(&frame_size), sizeof(int));
	ofs.write((const char*)(xyz.data), xyz.elemSize() * xyz.total());
	ofs.write((const char*)(flow.data), flow.elemSize() * flow.total());
}

int main(int argc, char* argv[]) {
	
    if (argc < 2) {
		return 0;
	}

	if (std::string(argv[1]) == "--capture") {
		bool buffer = argc > 2;

		//real time processing by default
		DeviceInterface *device = new RealSenseInterface(640, 480);
		device->start_device();

		FPSTimer timer(5);
		char key;
		Mat rgb, depth, xyz;
		while (key != 'c') {
			device->capture_frame(rgb, depth, xyz, buffer);
			cv::imshow("rgb", rgb);
			cv::imshow("depth", depth);
			cv::imshow("xyz", xyz);

			key = cv::waitKey(1);
			timer.pulse();
		}
		if (buffer)
			device->process_frame_buffer("db/" + std::string(argv[2]));

		device->stop_device();
		device->clear_frame_buffer();
		delete device;
	}


	if (std::string(argv[1]) == "--process") {
		//load from DB and process flow
		printf("loading buffer...\n");
		std::vector<cv::Mat*> intensity, depth, xyz;
		loadDB("db/" + std::string(argv[2]), intensity, depth, xyz);

		PD_flow_opencv sceneflow(240);
		sceneflow.initializeCUDA(640, 480);
		Mat i1, d1, i2, d2;

		cv::Mat display(960, 1280, CV_8UC3, Scalar(0, 0, 0));//display in RGB[0~1]
		//VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(1280, 960), true);
		std::ofstream ofs("data.bin", ios::binary);
		int num_frame = intensity.size() - 1;
		ofs.write((char *)&num_frame, sizeof(int));

		for (int i = 0; i < num_frame; ++i) {
			Mat flow, flow_color, xyz_res;

			i1 = *intensity[i];
			i2 = *intensity[i + 1];
			d1 = *depth[i];
			d2 = *depth[i + 1];

			sceneflow.loadRGBDFrames(i1, d1, i2, d2);
			sceneflow.solveSceneFlowGPU();
			sceneflow.getResult(flow, flow_color);
#if HIGH_RES
			cv::resize(flow, flow, cv::Size(640, 480)); //flow as vector
			xyz_res = *xyz[i];
#else
			cv::resize(*xyz[i], xyz_res, cv::Size(320, 240));
#endif
			cv::resize(flow_color, flow_color, cv::Size(640, 480)); //flow as color

			//write out xyz and flow as [320*240] binary array
			writeOut(ofs, xyz_res, flow);

			displayView(display, i1, d1, *xyz[i], flow_color);
			cv::imshow("display", display);
			//video.write(display);
			char key = cv::waitKey(1);
			if (key == 'c')
				break;
		}
		ofs.close();
		freeDB(intensity, depth, xyz);
		sceneflow.freeGPUMemory();

	}


	//create augmented reality tracking board
	if (std::string(argv[1]) == "--board") {
		ARScene ar;
		ar.create_board("board.png", "board.conf", 2, 3);
		return 0;
	}


    return 0;
}