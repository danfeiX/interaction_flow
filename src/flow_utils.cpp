#include "flow_utils.h"

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

inline Point3f operator < (const Point3f& p1, const Point3f& p2)
{
	return p1.x < p2.x && p1.y < p2.y && p1.z < p2.z;
}


//input: 3-channel Mat of size M x N, each channel contain x, y, z coordinate, respectively
//output: 1-channle Mat of size 3 x M*N, each row contain transformed x, y, z coordinate, respectively
//transform_mat: RT matrix of size 4 x 4
void transformPointCloud(cv::Mat & out_xyz, const cv::Mat & in_xyz, const cv::Mat & transform_mat)
{
	cv::Mat hg_pts; //homogeneous xyz points
	cv::Mat xyz_chans[4];

	//pad with 1's
	cv::split(in_xyz, xyz_chans);
	xyz_chans[3] = cv::Mat::ones(in_xyz.rows, in_xyz.cols, CV_32FC1);
	cv::merge(xyz_chans, 4, hg_pts);
	hg_pts = hg_pts.reshape(1, in_xyz.rows * in_xyz.cols);

	//transform
	out_xyz = transform_mat*hg_pts.t();
	out_xyz.convertTo(out_xyz, CV_32FC1);
}

void rotateFlowCloud(cv::Mat & out_flow, const cv::Mat & in_flow, const cv::Mat & transform_mat)
{
	Mat flow_reshaped;
	flow_reshaped = in_flow.reshape(1, in_flow.rows * in_flow.cols);

	//transform
	out_flow = transform_mat*flow_reshaped.t();
	out_flow.convertTo(out_flow, CV_32FC1);
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