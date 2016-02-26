#include "ARScene.h"

using namespace aruco;
using namespace cv;
int iThresParam1, iThresParam2;
double ThresParam1 = 7;
double ThresParam2 = 20;
cv::Mat im_in;

ARScene::ARScene()
{
}


void ARScene::init_detector(const cv::Mat& cam_mat, const cv::Mat& distort_mat, 
							const cv::Size im_size, const float marker_size/*in meter*/, const string board_fn/*board conf*/) {
	try{
		cam_mat.copyTo(m_cam_param.CameraMatrix);
		distort_mat.copyTo(m_cam_param.Distorsion);
		m_cam_param.CamSize.width = im_size.width;
		m_cam_param.CamSize.height = im_size.height;

		cout << board_fn << endl;
		cout << m_cam_param.CameraMatrix << endl;

		BInfo.readFromFile(board_fn);
		board_detector.setParams(BInfo, m_cam_param, marker_size);
		board_detector.getMarkerDetector().setCornerRefinementMethod(MarkerDetector::HARRIS);
		board_detector.getMarkerDetector().setThresholdParams(ThresParam1, ThresParam2);
		board_detector.set_repj_err_thres(1.5);
	} catch (std::exception &ex) {
		cv::waitKey(0);
		cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		cout << "Exception :" << ex.what() << endl;
	}
}


void ARScene::create_board(const string im_fn, const string bc_fn, const int XSize, const int YSize) {

	int pixSize = 400;
	float interMarkerDistance = 0.2;
	bool isChessBoard = false;
	int typeBoard = 0;

	interMarkerDistance = 0.05;
	cv::Mat BoardImage;
	BoardImage = aruco::FiducidalMarkers::createBoardImage(cv::Size(XSize, YSize), pixSize, pixSize * interMarkerDistance, BInfo);
	//BoardImage = aruco::FiducidalMarkers::createBoardImage_ChessBoard(Size(XSize, YSize), pixSize, BInfo);
	//BoardImage = aruco::FiducidalMarkers::createBoardImage_Frame(Size(XSize, YSize), pixSize, pixSize * interMarkerDistance, BInfo);
	imwrite(im_fn, BoardImage);
	BInfo.saveToFile(bc_fn);
}


bool ARScene::estimateBoardPose(cv::Mat &im, cv::Mat &Rvec, cv::Mat &Tvec) {
	bool success = false;
	im.copyTo(im_in);
	try {
		float probDetect = board_detector.detect(im_in);

		// print marker borders
		for (unsigned int i = 0; i < board_detector.getDetectedMarkers().size(); i++)
			board_detector.getDetectedMarkers()[i].draw(im_in, cv::Scalar(0, 0, 255), 1);


		// print board
		if (m_cam_param.isValid()) {
			if (probDetect > 0.2) {
				Board &board = board_detector.getDetectedBoard();
				board.Rvec.copyTo(Rvec);
				board.Tvec.copyTo(Tvec);
				CvDrawingUtils::draw3dAxis(im_in, board, m_cam_param);
				CvDrawingUtils::draw3dCube(im_in, board, m_cam_param);
				//draw3dBoardCube(im_in, board, m_cam_param);
				success = true;
			}
		}

		// show input with augmented information and  the thresholded image
		cv::imshow("thresh", board_detector.getMarkerDetector().getThresholdedImage());

	}
	catch (std::exception &ex) {
		cout << "Exception :" << ex.what() << endl;
	}

	cv::imshow("in", im_in);
	return success;
}

void ARScene::cvTackBarEvents(int pos, void *) {
	if (iThresParam1 < 3)
		iThresParam1 = 3;
	if (iThresParam1 % 2 != 1)
		iThresParam1++;
	if (ThresParam2 < 1)
		ThresParam2 = 1;
	ThresParam1 = iThresParam1;
	ThresParam2 = iThresParam2;
	board_detector.getMarkerDetector().setThresholdParams(ThresParam1, ThresParam2);

}


ARScene::~ARScene()
{
}
