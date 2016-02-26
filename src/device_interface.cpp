#include "device_interface.h"
#include "flow_utils.h"
DeviceInterface::DeviceInterface(size_t w, size_t h, bool buffer, size_t device_id) :
m_device_id(device_id),
curr_rgb(nullptr),
curr_xyz(nullptr),
curr_depth(nullptr),
im_width(w),
im_height(h),
do_buffer(buffer),
m_thread(),
thread_running(false)
{
	m_cam_to_world = cv::Mat::eye(4, 4, CV_32FC1);
}

DeviceInterface::DeviceInterface():
DeviceInterface(640, 480, false, 0)
{
}


DeviceInterface::~DeviceInterface()
{
}

void DeviceInterface::update(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz)
{
	if (!do_buffer) {
		rgb_buffer.clear();
		depth_buffer.clear();
		xyz_buffer.clear();
	}
	rgb_buffer.push_back(rgb);
	depth_buffer.push_back(depth);
	xyz_buffer.push_back(xyz);

	curr_rgb = &rgb_buffer.back();
	curr_depth = &depth_buffer.back();
	curr_xyz = &xyz_buffer.back();
}

bool DeviceInterface::get_latest_frame(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz)
{
	m_lock.lock();
	bool success = true;
	if (!curr_rgb || !curr_depth || !curr_xyz) {
		success = false;
	}
	else {
		curr_rgb->copyTo(rgb);
		curr_depth->copyTo(depth);
		curr_xyz->copyTo(xyz);
	}
	m_lock.unlock();
	return success;
}

void DeviceInterface::spawn_capture_thread()
{
	thread_running = true;
	m_thread = std::thread(&DeviceInterface::capture_callback, this);
}

void DeviceInterface::stop_capture()
{
	thread_running = false;
	if (m_thread.joinable()) m_thread.join();
}

void DeviceInterface::capture_callback()
{
	FPSTimer timer("CAPTURE_LOOP", 5);
	while (thread_running) 
	{
		capture_frame();
		timer.pulse();
	}
}

void DeviceInterface::loadCalibrationFile(const std::string file_name)
{

}

void DeviceInterface::calibrateCameraPose(const size_t num_frames)
{
	char op_key;
	bool do_calib = false;
	size_t frame_count = 0;
	vector<Mat> Rvecs, Tvecs;

	while (frame_count < num_frames)
	{
		if (!thread_running) {
			capture_frame();
		}

		Mat rgb, depth, xyz;
		if (!get_latest_frame(rgb, depth, xyz))
			continue;

		op_key = cv::waitKey(1);
		if (op_key == 'k') { //begin collect calibration data
			cout << "begin calibration..." << endl;
			do_calib = true;
		}


		Mat Rvec, Tvec;
		bool success = ar.estimateBoardPose(rgb, Rvec, Tvec);

		if (do_calib && success) {
			Rvecs.push_back(Mat());
			Tvecs.push_back(Mat());
			Rvec.copyTo(Rvecs.back());
			Tvec.copyTo(Tvecs.back());
			frame_count++;

		}
	}

	Mat rvec_mean = Rvecs[0].clone();
	Mat tvec_mean = Tvecs[0].clone();
	for (int i = 1; i < num_frames; ++i) {
		rvec_mean += Rvecs[i];
		tvec_mean += Tvecs[i];
	}
	rvec_mean /= float(num_frames);
	tvec_mean /= float(num_frames);
	Mat R;//rotation matrix
	Rodrigues(rvec_mean, R);

	m_cam_to_world = Mat::zeros(4, 4, CV_32FC1);
	m_cam_to_world.at<float>(3, 3) = 1;
	R.copyTo(m_cam_to_world(Rect(0, 0, 3, 3))) ;
	tvec_mean.copyTo(m_cam_to_world(Rect(3, 0, 1, 3)));

	float theta = M_PI / 2;
	float rotX[4][4] = {
		{ 1, 0, 0, 0 },
		{ 0, cos(theta), -sin(theta), 0 },
		{ 0, sin(theta), cos(theta), 0 },
		{ 0, 0, 0, 1 }
	};
	theta = -M_PI / 2;
	float rotY[4][4] = {
		{ cos(theta), 0, sin(theta), 0 },
		{ 0, 1, 0, 0 },
		{ -sin(theta), 0, cos(theta), 0 },
		{ 0, 0, 0, 1 }
	};

	Mat rotXM(4, 4, CV_32FC1, rotX);
	Mat rotYM(4, 4, CV_32FC1, rotY);
	m_cam_to_world = rotYM * rotXM * m_cam_to_world.inv();

	cout << "Calibration completed" << endl;
	cout << "mean rvec: " << rvec_mean << endl;
	cout << "mean tvec: " << tvec_mean << endl;
	cout << "cam_to_world: " << m_cam_to_world << endl;
}

//1. Converts the rgb and depth frame buffer to cv::Mat that is of
//the input format of PD-flow.
//2. Writes the cv::Mat to a binary file
void DeviceInterface::process_frame_buffer(const std::string filename) {
	m_lock.lock();
	cout << "processing buffer..." << endl;
	ofstream output_file(filename.c_str(), ios::binary);
	size_t num_frame = num_frames();
	output_file.write((char *)&num_frame, sizeof(size_t));

	for (int i = 0; i < num_frames(); ++i) {
		Mat intensity;
		writeMatBinary(output_file, rgb_buffer[i]);
		writeMatBinary(output_file, depth_buffer[i]);
		writeMatBinary(output_file, xyz_buffer[i]);

		cv::imshow("depth2rgb", depth_buffer[i]);
		cv::imshow("intensity", xyz_buffer[i]);
		int key = cv::waitKey(1);
	}
	m_lock.unlock();
}
