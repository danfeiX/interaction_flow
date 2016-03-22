#ifndef FLOW_UTILS_H
#define FLOW_UTILS_H
#include <opencv2/opencv.hpp>
#include <fstream>
#include <time.h>

using namespace cv;
using namespace std;

void threshPosition(cv::Mat& depth, cv::Mat & xyz, const cv::Point3f min, const cv::Point3f max);
bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);
void transformPointCloud(cv::Mat & out_xyz, const cv::Mat & in_xyz, const cv::Mat & transform_mat);
void rotateFlowCloud(cv::Mat & out_flow , const cv::Mat & in_flow, const cv::Mat & transform_mat);


// write a vector into a binary file
template<typename T>
void writeVector(std::ofstream& ofs, const vector<T> & out)
{
	int s = out.size();
	ofs.write((char*)(&s), sizeof(int));
	ofs.write((char*)(&out[0]), s*sizeof(T));
}


// read in a vector from a binary file
template<typename T>
void readVector(std::ifstream& ifs, vector<T> & in)
{
	int s;
	ifs.read((char*)(&s), sizeof(int));
	in.resize(s);
	ifs.read((char*)(&in[0]), s*sizeof(T));
}

class FPSTimer 
{
public:
	FPSTimer(std::string t_name, float freq):name(t_name), print_freq(freq) {
	}

	void pulse() {
		count++;
		float sec = (clock() - start_t) / CLOCKS_PER_SEC;
		if (sec > print_freq) {
			cout <<name<< " FPS: " << (float)count / sec << endl;
			start_t = clock();
			count = 0;
		}
	}
private:
	size_t count;
	float start_t;
	float print_freq;
	std::string name;
};
#endif