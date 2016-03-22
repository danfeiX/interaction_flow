#include "realsense_interface.h"

#define ALIGN_TO_COLOR 1

using namespace cv;
RealSenseInterface::RealSenseInterface(size_t w, size_t h, size_t device_id, rs::context *ctx) :
DeviceInterface(w, h, device_id),
m_ctx(ctx)
{
}


RealSenseInterface::RealSenseInterface()
{
}


RealSenseInterface::~RealSenseInterface()
{
}


void RealSenseInterface::get_frame(cv::Mat& frame, const rs::stream stream, int *timestamp)
{
	assert(m_dev->is_stream_enabled(stream));
	
	*timestamp = m_dev->get_frame_timestamp(stream); //get timestamp
	const void* data = m_dev->get_frame_data(stream); //get data
	int width = m_dev->get_stream_width(stream); //get frame width
	int height = m_dev->get_stream_height(stream); //get frame height
	rs::format format = m_dev->get_stream_format(stream); //get data format
	
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


void RealSenseInterface::capture_frame()
{
	Mat depth_int;
	int rgb_ts, depth_ts, xyz_ts;
	m_dev->wait_for_frames();

	Mat rgb, depth;
#ifdef ALIGN_TO_COLOR //Align depth and xyz to color image
	get_frame(rgb, rs::stream::rectified_color, &rgb_ts);
	get_frame(depth_int, rs::stream::depth_aligned_to_rectified_color, &depth_ts);
	depth_int.convertTo(depth, CV_32FC1);
	depth *= m_dev->get_depth_scale(); //convert to meter
	//depth_to_xyz(depth, xyz);
#else //Align color to depth and xyz
	get_frame(rgb, rs::stream::color_aligned_to_depth, &rgb_ts);
	get_frame(depth_int, rs::stream::depth, &depth_ts);
	get_frame(xyz, rs::stream::points, &xyz_ts);
	depth_int.convertTo(depth, CV_32FC1);
	depth *= dev->get_depth_scale(); //convert to meter
#endif
	update(rgb, depth, rgb_ts);
}


//Transform color image space to depth space
void RealSenseInterface::color_to_depth(const cv::Mat& color, cv::Mat& outDepth)
{
	//TODO
	//m_extrinsics.transform()
	rs::extrinsics extrinsics = m_dev->get_extrinsics(rs::stream::rectified_color, rs::stream::depth);
}


void RealSenseInterface::init_ar(const float marker_size, const string board_fn) {
	float cat_data[9] = { m_intrinsics.fx, 0, m_intrinsics.ppx, 0, m_intrinsics.fy, m_intrinsics.ppy, 0, 0, 1 };
	float dist_data[4] = { m_intrinsics.coeffs[0], //k1
						   m_intrinsics.coeffs[0], //k2
						   m_intrinsics.coeffs[0], //p1
						   m_intrinsics.coeffs[0]  //p2
						 };

	cv::Mat cam_mat = cv::Mat(3, 3, CV_32F, cat_data);
	cv::Mat distortion = cv::Mat(4, 1, CV_32F, dist_data);

	ar.init_detector(cam_mat, distortion, cv::Size(im_width, im_height), marker_size, board_fn);
}


void RealSenseInterface::start_device() 
{
	if (m_ctx->get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");

	m_dev = m_ctx->get_device(m_device_id);

	//enable all three streams

	try {
		m_dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
		m_dev->enable_stream(rs::stream::color, rs::preset::best_quality);
		m_dev->enable_stream(rs::stream::infrared2, rs::preset::best_quality);
	}
	catch (rs::error e) {
		std::cout << e.what() << endl;
	}

	m_intrinsics = m_dev->get_stream_intrinsics(rs::stream::rectified_color);
	fx = m_intrinsics.fx;
	fy = m_intrinsics.fy;
	cx = m_intrinsics.ppx;
	cy = m_intrinsics.ppy;

	//start the device
	m_dev->start();
}


void RealSenseInterface::stop_device() 
{
	m_dev->stop();
}