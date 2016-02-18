#include "flow_utils.h"
using namespace std;
//Abstract class of device interface
void threshPosition(cv::Mat& depth, cv::Mat & xyz, const cv::Point3f min, const cv::Point3f max)
{
	/*
	Invalidate points that are not in a box region
	*/
	const float bad_val = std::numeric_limits<float>::quiet_NaN();
	const cv::Point3f bad_pos(bad_val, bad_val, bad_val);
	for (int r = 0; r < xyz.rows; ++r) {
		for (int c = 0; c < xyz.cols; ++c) {
			if (xyz.at<cv::Point3f>(r, c).x > max.x || xyz.at<cv::Point3f>(r, c).x < min.x ||
				xyz.at<cv::Point3f>(r, c).y > max.y || xyz.at<cv::Point3f>(r, c).y < min.y ||
				xyz.at<cv::Point3f>(r, c).z > max.z || xyz.at<cv::Point3f>(r, c).z < min.z) {
				depth.at<float>(r, c) = bad_val;
				xyz.at<cv::Point3f>(r, c) = bad_pos;
			}
		}
	}
}

//! Write cv::Mat to binary
/*!
\param[in] ofs output file stream
\param[out] in_mat mat to load
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