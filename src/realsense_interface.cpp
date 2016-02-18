#include "realsense_interface.h"
#include "flow_utils.h"

using namespace cv;
RealSenseInterface::RealSenseInterface(const size_t im_w, const size_t im_h)
{
	im_width = im_w;
	im_height = im_h;
}

RealSenseInterface::RealSenseInterface()
{
	im_width = 640;
	im_height = 480;
}


RealSenseInterface::~RealSenseInterface()
{
}


void RealSenseInterface::get_frame(cv::Mat& frame, rs::stream stream, int *timestamp)
{
	assert(dev->is_stream_enabled(stream));
	
	*timestamp = dev->get_frame_timestamp(stream); //get timestamp
	const void* data = dev->get_frame_data(stream); //get data
	int width = dev->get_stream_width(stream); //get frame width
	int height = dev->get_stream_height(stream); //get frame height
	rs::format format = dev->get_stream_format(stream); //get data format
	
	Mat tmp;
	switch (format)
	{
	case rs::format::z16: case rs::format::disparity16:
		frame = cv::Mat(height, width, CV_16UC1, (uchar*)data);
		break;
	case rs::format::rgb8: 
		tmp = cv::Mat(height, width, CV_8UC3, (uchar*)data);
		cvtColor(tmp, frame, COLOR_RGB2BGR);
		break;
	case rs::format::bgr8: 
		frame = cv::Mat(height, width, CV_8UC3, (uchar*)data);
		break;
	case rs::format::rgba8: 
		frame = cv::Mat(height, width, CV_8UC4, (uchar*)data);
		cvtColor(frame, frame, COLOR_RGBA2BGRA);
		break;
	case rs::format::bgra8:
		frame = cv::Mat(height, width, CV_8UC4, (uchar*)data);
		break;
	case rs::format::xyz32f:
		frame = cv::Mat(height, width, CV_32FC3, (uchar*)data);
		break;
	default:
		throw std::runtime_error("Unimplemented frame format");
		break;
	}

}

void RealSenseInterface::capture_frame(cv::Mat & rgb, cv::Mat & depth, cv::Mat & xyz, bool buffer)
{
	Mat depth_int;
	int rgb_ts, depth_ts, xyz_ts;

	dev->wait_for_frames();

	//get_frame(rgb, rs::stream::rectified_color, &rgb_ts);
	//get_frame(depth_int, rs::stream::depth_aligned_to_rectified_color, &depth_ts);
	get_frame(rgb, rs::stream::color_aligned_to_depth, &rgb_ts);
	get_frame(depth_int, rs::stream::depth, &depth_ts);
	get_frame(xyz, rs::stream::points, &xyz_ts);

	depth_int.convertTo(depth, CV_32FC1);
	depth *= dev->get_depth_scale(); //convert to meter

	if (buffer) {
		Mat rgb_, depth_, xyz_;
		rgb.copyTo(rgb_);
		depth.copyTo(depth_);
		xyz.copyTo(xyz_);
		rgb_buffer.push_back(rgb_);
		depth_buffer.push_back(depth_);
		xyz_buffer.push_back(xyz_);
	}
}

void RealSenseInterface::start_device() 
{
	if (ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

	dev = ctx.get_device(0);

	//enable all three streams
	dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
	dev->enable_stream(rs::stream::color, rs::preset::best_quality);
	try { dev->enable_stream(rs::stream::infrared2, rs::preset::best_quality); }
	catch (...) {}

	//start the device
	dev->start();
}

void RealSenseInterface::stop_device() 
{
	dev->stop();
}

void RealSenseInterface::clear_frame_buffer() 
{

}

void RealSenseInterface::process_frame_buffer(std::string filename) 
{
	cout << "processing buffer..." << endl;
	ofstream output_file(filename.c_str(), ios::binary);
	size_t num_frame = num_frames();
	output_file.write((char *)&num_frame, sizeof(size_t));

	for (int i = 0; i < num_frames(); ++i) {
		Mat intensity;
		cvtColor(rgb_buffer[i], intensity, COLOR_BGR2GRAY);
		writeMatBinary(output_file, intensity);
		writeMatBinary(output_file, depth_buffer[i]);
		writeMatBinary(output_file, xyz_buffer[i]);

		cv::imshow("depth2rgb", depth_buffer[i]);
		cv::imshow("intensity", xyz_buffer[i]);
		int key = cv::waitKey(1);
	}
}

void RealSenseInterface::init_ar(const float marker_size, const string board_fn) 
{

}

size_t RealSenseInterface::num_frames() 
{
	assert(rgb_buffer.size() == depth_buffer.size() == xyz_buffer.size());
	return rgb_buffer.size();
}