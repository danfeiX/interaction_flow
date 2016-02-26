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
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <fstream>
#include "device_interface.h"
#define BOUND_MIN_X -1
#define BOUND_MIN_Y -1
#define BOUND_MIN_Z 0
#define BOUND_MAX_X 1
#define BOUND_MAX_Y 0.75
#define BOUND_MAX_Z 1.5


using namespace libfreenect2;
class KinectInterface : public DeviceInterface{
public:
	enum Processor { cl, gl, cpu };
	KinectInterface(size_t w, size_t h, bool buffer, size_t device_id);
	KinectInterface();
    ~KinectInterface();

	virtual void capture_frame();
	virtual void start_device(); //start the kinect device using LibFreenect2
    virtual void stop_device(); //stop the device
	virtual void clear_frame_buffer(); //clear the frame buffer
	virtual void init_ar(const float marker_size, const string board_fn);
	virtual size_t num_frames();

#if 0
	void save_buffer(std::string filename);
	void load_buffer(std::string filename);
#endif // 0


private:
	void register_depth(const Frame*, const Frame*, cv::Mat &, cv::Mat &); //process individual frame
	void depth_to_xyz(const cv::Mat& rectified_depth, cv::Mat& outXYZ);

	Processor backend;

    SyncMultiFrameListener* listener;
    Registration* registration;
    Frame* undistorted;
    Frame* registered;
    Frame* depth2rgb;
    Freenect2Device *dev;
    Freenect2 freenect2;

};
#endif //INTERACTION_FLOW_KINECT_INTERFACE_H
