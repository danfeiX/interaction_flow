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

void displayFlow()

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
		Mat i1, d1, i2, d2, flow, flow_res;

		cv::Mat display(960, 1280, CV_32FC3, Scalar(0, 0, 0));

		for (int i = 0; i < rgb.size()-1; ++i) {
			i1 = *intensity[i];
			i2 = *intensity[i + 1];
			d1 = *depth[i];
			d2 = *depth[i + 1];
			sceneflow.loadRGBDFrames(i1, d1, i2, d2);
			sceneflow.solveSceneFlowGPU();
			sceneflow.getResult(flow);

			cv::resize(flow, flow_res, cv::Size(640, 480));
			Mat i1_, d1_, xyz_;
			cv::cvtColor(i1, i1_, COLOR_GRAY2RGB);
			i1_.convertTo(i1_, CV_32FC3);
			i1_ = i1_ / 255;
			cv::cvtColor(d1, d1_, COLOR_GRAY2RGB);
			i1_.copyTo(display(cv::Rect(0, 0, 640, 480)));
			d1_.copyTo(display(cv::Rect(640, 0, 640, 480)));


			xyz[i]->copyTo(display(cv::Rect(640, 480, 640, 480)));
			flow_res.convertTo(flow_res, CV_32FC3);
			flow_res = flow_res / 255;
			flow_res.copyTo(display(cv::Rect(0, 480, 640, 480)));
			//video.write(display);
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