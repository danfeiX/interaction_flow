//
// Created by danfei on 1/22/16.
//

#ifndef INTERACTION_FLOW_KINECT_INTERFACE_H
#define INTERACTION_FLOW_KINECT_INTERFACE_H
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>

enum Processor { cl, gl, cpu};
bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);

using namespace libfreenect2;
class KinectInterface {
public:
    KinectInterface();
    void capture_frame();
    ~KinectInterface();
	void start_device(Processor backend);
    void stop_device();
    void clear_buffer();
	void postprocess_buffer(std::string filename);
	void process_frame_PDFlow(const Frame*, const Frame*, cv::Mat &, cv::Mat &, cv::Mat &);
	void process_frame_PDFlow(size_t frameID, cv::Mat&, cv::Mat&, cv::Mat &);
	void process_frame_XYZ(const cv::Mat& rectified_depth, cv::Mat& outXYZ);
	size_t num_frames() { return rgb_frame_buffer.size(); }

	//deprecated, don't use
	void save_buffer(std::string filename);
	void load_buffer(std::string filename);

private:
    std::vector<Frame*> rgb_frame_buffer;
    std::vector<Frame*> depth_frame_buffer;

    SyncMultiFrameListener* listener;
    Registration* registration;
    Frame* undistorted;
    Frame* registered;
    Frame* depth2rgb;
    Freenect2Device *dev;
    Freenect2 freenect2;
};
#endif //INTERACTION_FLOW_KINECT_INTERFACE_H
