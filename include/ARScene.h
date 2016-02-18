#pragma once
#include <aruco/aruco.h>
#include <aruco/highlyreliablemarkers.h>
#include <aruco/arucofidmarkers.h>
#include <aruco/board.h>
#include <aruco/boarddetector.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
using namespace aruco;
class ARScene
{
public:
	ARScene();
	void init_detector(const cv::Mat& cam_mat, const cv::Mat& distort_mat,
					   const cv::Size im_size, const float marker_size, const string board_fn);
	void create_board(const string im_fn, const string bc_fn, const int XSize, const int YSize);
	void detect_board(cv::Mat &im);
	~ARScene();
private:
	MarkerDetector m_detector;
	CameraParameters m_cam_param;
	float marker_size;
	aruco::BoardConfiguration BInfo;
	BoardDetector board_detector;
	void ARScene::cvTackBarEvents(int pos, void *);
};

