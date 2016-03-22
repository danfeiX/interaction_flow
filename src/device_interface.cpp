#include "device_interface.h"
DeviceInterface::DeviceInterface(size_t w, size_t h, size_t device_id) :
m_device_id(device_id),
im_width(w),
im_height(h),
do_buffer(false),
m_thread(),
m_thread_running(false)
{
	m_cam_to_world = cv::Mat::eye(4, 4, CV_32FC1);
	cx = cy = fx = fy = 0;
}


DeviceInterface::DeviceInterface():
DeviceInterface(640, 480, 0)
{
}


DeviceInterface::~DeviceInterface()
{
}


void DeviceInterface::start_all()
{
	start_device();
	spawn_capture_thread();
	init_ar(MARKER_SIZE, "D:/workspace/interaction_flow/ar/board.conf");
}


void DeviceInterface::stop_all()
{
	stop_capture_thread();
	stop_device();
	clear_frame_buffer();
}


void DeviceInterface::spawn_capture_thread()
{
	m_thread_running = true;
	m_thread = std::thread(&DeviceInterface::capture_callback, this);
}


void DeviceInterface::stop_capture_thread()
{
	m_thread_running = false;
	if (m_thread.joinable()) m_thread.join();
}


void DeviceInterface::capture_callback()
{
	FPSTimer timer("CAPTURE_LOOP", 5);
	while (m_thread_running)
	{
		capture_frame();
		timer.pulse();
	}
}


void DeviceInterface::update(const cv::Mat & rgb, const cv::Mat & depth, const int timestamp)
{
	scope_guard sg(m_lock);

	if (do_buffer) {
		RGBDFrame new_frame(rgb, depth, timestamp);
		new_frame.cx = cx;
		new_frame.cy = cy;
		new_frame.fx = fx;
		new_frame.fy = fy;
		rgbd_buffer.push_back(new_frame);
		printf("buffered = %i\n", rgbd_buffer.size());
	}
	//update container of the latest frame
	rgb.copyTo(curr_rgb);
	depth.copyTo(curr_depth);
}


bool DeviceInterface::get_latest_frame(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz)
{
	bool success = true;
	lock_thread();
	{
		if (curr_rgb.dims == 0) {
			success = false;
		}
		else {
			curr_rgb.copyTo(rgb);
			curr_depth.copyTo(depth);
		}
	}
	unlock_thread();

	if (success) {
		depth_to_xyz(depth, xyz);
	}

	return success;
}


void DeviceInterface::save_raw_frames(const std::string filename)
{
	scope_guard sg(m_lock);
	{
		cout << "saving raw frames..." << endl;
		ofstream output_file(filename.c_str(), ios::binary);
		size_t num_frame = num_frames();
		output_file.write((char *)&num_frame, sizeof(size_t));

		for (int i = 0; i < num_frame; ++i) {
			output_file.write((char*)(&rgbd_buffer[i]), sizeof(RGBDFrame));
			writeMatBinary(output_file, rgbd_buffer[i].rgb);
			writeMatBinary(output_file, rgbd_buffer[i].depth);

			cv::imshow("rgb", rgbd_buffer[i].rgb);
			cv::imshow("depth", rgbd_buffer[i].depth);

			int key = cv::waitKey(1);
		}
	}
}


size_t DeviceInterface::num_frames()
{
	return rgbd_buffer.size();
}


void DeviceInterface::clear_frame_buffer()
{
	rgbd_buffer.clear();
}


void DeviceInterface::depth_to_xyz(const cv::Mat& rectified_depth, cv::Mat& outXYZ)
{
	const float bad_val = std::numeric_limits<float>::quiet_NaN();
	const cv::Point3f bad_pos(bad_val, bad_val, bad_val);
	outXYZ = cv::Mat(rectified_depth.rows, rectified_depth.cols, CV_32FC3);

	for (int r = 0; r < rectified_depth.rows; ++r) {
		for (int c = 0; c < rectified_depth.cols; ++c) {
			float depth_val = rectified_depth.at<float>(r, c);
			if (isnan(depth_val) || depth_val <= 0.001)
			{
				//depth value is not valid
				outXYZ.at<cv::Point3f>(r, c) = bad_pos;
			}
			else {
				float x = (c + 0.5 - cx) / fx * depth_val;
				float y = (r + 0.5 - cy) / fy * depth_val;
				float z = depth_val;
				outXYZ.at<cv::Point3f>(r, c) = cv::Point3f(x, y, z);
			}
		}
	}
}


void DeviceInterface::loadCalibrationFile(const std::string file_name)
{
	ifstream in(file_name, ios::binary);
	readMatBinary(in, m_cam_to_world);
}


void DeviceInterface::calibrateCameraPose(const size_t num_frames)
{
	char op_key;
	bool do_calib = false;
	size_t frame_count = 0;
	vector<Mat> Rvecs, Tvecs;

	while (frame_count < num_frames)
	{
		if (!m_thread_running) {
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
	R.copyTo(m_cam_to_world(Rect(0, 0, 3, 3)));
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
	m_cam_to_world = rotYM * rotXM * m_cam_to_world.inv(); //convert to OpenGL coordinate system

	cout << "Calibration completed" << endl;
	cout << "mean rvec: " << rvec_mean << endl;
	cout << "mean tvec: " << tvec_mean << endl;
	cout << "cam_to_world: " << m_cam_to_world << endl;

	ofstream out("D:/workspace/interaction_flow/ar/camera_pose.mat", ios::binary);
	writeMatBinary(out, m_cam_to_world);
}