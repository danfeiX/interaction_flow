#include <iostream>

#include "kinect/kinect_interface.h"
#include "scene_flow.h"
#include <signal.h>
#include <time.h>
//#include "scene_flow.h"

using namespace std;
bool protonect_shutdown = false; // Whether the running application should shut down.
void sigint_handler(int s)
{
    protonect_shutdown = true;

}
using namespace std;
using namespace cv;


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

void threshPosition(cv::Mat& depth, cv::Mat & xyz, const cv::Point3f min, const cv::Point3f max) {
	/*
	Invalidate points that are not in a box region
	*/
	const float bad_val = std::numeric_limits<float>::quiet_NaN();
	const cv::Point3f bad_pos(bad_val, bad_val, bad_val);
	for (int r = 0; r < xyz.rows; ++r)
		for (int c = 0; c < xyz.cols; ++c) {
			if (xyz.at<cv::Point3f>(r, c).x > max.x || xyz.at<cv::Point3f>(r, c).x < min.x ||
				xyz.at<cv::Point3f>(r, c).y > max.y|| xyz.at<cv::Point3f>(r, c).y < min.y ||
				xyz.at<cv::Point3f>(r, c).z > max.z || xyz.at<cv::Point3f>(r, c).z < min.z) {
				depth.at<float>(r, c) = bad_val;
				xyz.at<cv::Point3f>(r,c) = bad_pos;
			}
		}
}

void loadDB(const std::string filename, std::vector<cv::Mat*>& rgb_v, std::vector<cv::Mat*>& intensity_v, std::vector<cv::Mat*>& depth_v, std::vector<cv::Mat*>& xyz_v) {
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
		readMatBinary(input_file, *rgb);
		readMatBinary(input_file, *depth);
		readMatBinary(input_file, *xyz);
		threshPosition(*depth, *xyz, cv::Point3f(-1, -1, 0), cv::Point3f(2, 0.75, 1.5));

		rgb_v.push_back(rgb);
		intensity_v.push_back(intensity);
		depth_v.push_back(depth);
		xyz_v.push_back(xyz);
	}
}

void freeDB(std::vector<cv::Mat*>& rgb_v, std::vector<cv::Mat*>& intensity_v, std::vector<cv::Mat*>& depth_v, std::vector<cv::Mat*>& xyz_v) {
	for (int i = 0; i < rgb_v.size(); ++i) {
		delete rgb_v[i];
		delete intensity_v[i];
		delete depth_v[i];
		delete xyz_v[i];
	}
}

void displayView(cv::Mat& display, cv::Mat & intensity, cv::Mat &depth, cv::Mat& xyz, cv::Mat &flow_color) {
	Mat i1_, d1_, xyz_;
	cv::cvtColor(intensity, i1_, COLOR_GRAY2RGB);
	i1_.convertTo(i1_, CV_32FC3);
	i1_ = i1_ / 255;
	cv::cvtColor(depth, d1_, COLOR_GRAY2RGB);
	i1_.copyTo(display(cv::Rect(0, 0, 640, 480)));
	d1_.copyTo(display(cv::Rect(640, 0, 640, 480)));
	xyz.copyTo(display(cv::Rect(640, 480, 640, 480)));
	flow_color.copyTo(display(cv::Rect(0, 480, 640, 480)));
}

int main(int argc, char* argv[]) {

	signal(SIGINT, sigint_handler);
    if (argc < 2) {
        return 0;
    }
    if (std::string(argv[1]) == "--record") {
		KinectInterface kinect;
		kinect.start_device(Processor::cl);
		int c = 0;
		clock_t t;
		int f;
		t = clock();
		char key;
		while (key!='c') {
			kinect.capture_frame();
			key = cv::waitKey(1);
			c++;
			float sec = (clock() - t) / CLOCKS_PER_SEC;
			if (sec > 10) {
				cout << "Record FPS: " << (float)c / sec << endl;
				t = clock();
				c = 0;
			}
		}
		cout << "Pose processing..." << endl;
		kinect.postprocess_buffer(argv[2]);
		cout << "Pose processing done" << endl;
		kinect.clear_buffer();
		cv::waitKey(1000000);

    } else if (std::string(argv[1]) == "--process") {
		printf("loading buffer...\n");
		std::vector<cv::Mat*> rgb, intensity, depth, xyz;
		loadDB(argv[2], rgb, intensity, depth, xyz);

		PD_flow_opencv sceneflow(240);
		sceneflow.initializeCUDA(640, 480);
		Mat i1, d1, i2, d2;

		cv::Mat display(960, 1280, CV_32FC3, Scalar(0, 0, 0));//display in RGB[0~1]

		for (int i = 0; i < rgb.size()-1; ++i) {
			Mat flow, flow_color;
			i1 = *intensity[i];
			i2 = *intensity[i + 1];
			d1 = *depth[i];
			d2 = *depth[i + 1];
			sceneflow.loadRGBDFrames(i1, d1, i2, d2);
			sceneflow.solveSceneFlowGPU();
			sceneflow.getResult(flow, flow_color);

			cv::resize(flow, flow, cv::Size(640, 480)); //flow as vector
			cv::resize(flow_color, flow_color, cv::Size(640, 480)); //flow as color
			displayView(display, i1, d1, *xyz[i], flow_color);
			cv::imshow("display", display);

			char key = cv::waitKey(1);
			if (protonect_shutdown || key == 'c')
				break;
		}

		freeDB(rgb, intensity, depth, xyz);
		sceneflow.freeGPUMemory();

    }

    return 0;
}