#include <opencv2/opencv.hpp>
#include <fstream>
#include <time.h>

using namespace cv;
using namespace std;

void threshPosition(cv::Mat& depth, cv::Mat & xyz, const cv::Point3f min, const cv::Point3f max);
bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);


class FPSTimer 
{
public:
	FPSTimer(float freq) {
		print_freq = freq;
	}
	void pulse() {
		count++;
		float sec = (clock() - start_t) / CLOCKS_PER_SEC;
		if (sec > print_freq) {
			cout << "FPS: " << (float)count / sec << endl;
			start_t = clock();
			count = 0;
		}
	}
private:
	size_t count;
	float start_t;
	float print_freq;
};