//
// Created by danfei on 1/22/16.
//
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include "kinect_interface.h"
using namespace std;

KinectInterface::KinectInterface(Processor backend) {

    //! [context]
    dev = nullptr;
    registration = nullptr;
    libfreenect2::PacketPipeline *pipeline = nullptr;
    //! [context]

    //! [discovery]
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;
    //! [discovery]

    int depthProcessor = backend;

    if(depthProcessor == Processor::cpu)
    {
        if(!pipeline)
            //! [pipeline]
            pipeline = new libfreenect2::CpuPacketPipeline();
        //! [pipeline]
    } else if (depthProcessor == Processor::gl) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if(!pipeline)
            pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    } else if (depthProcessor == Processor::cl) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!pipeline) {
            pipeline = new libfreenect2::OpenCLPacketPipeline();
        } else {
            std::cout << "Using OpenCL backend!" << std::endl;

        }
#else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }

    if(pipeline)
    {
        //! [open]
        dev = freenect2.openDevice(serial, pipeline);
        //! [open]
    } else {
        dev = freenect2.openDevice(serial);
    }

    if(!dev)
    {
        std::cout << "failure opening device!" << std::endl;
    }

    //! [listeners]
    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
                                                        libfreenect2::Frame::Depth);
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);

    //frame buffers
    undistorted = new libfreenect2::Frame(512, 424, 4);
    registered = new libfreenect2::Frame(512, 424, 4);
    depth2rgb = new libfreenect2::Frame(1920, 1080+2, 4);

    start_device();
}


void KinectInterface::start_device() {
    cout <<"starting device\n" <<endl;
    dev->start();
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

void KinectInterface::stop_device() {
    dev->stop();
}

void KinectInterface::capture_frame() {
    libfreenect2::FrameMap frame;
    listener->waitForNewFrame(frame);
    rgb_buffer.push_back(frame[libfreenect2::Frame::Color]);
    depth_buffer.push_back(frame[libfreenect2::Frame::Depth]);

}


void KinectInterface::save_buffer(std::string filename) {
    ofstream output_file(filename.c_str(), ios::binary);
    size_t num_frame = rgb_buffer.size();
    output_file.write((char *)&num_frame, sizeof(size_t));
    libfreenect2::Frame *frame;
    for (int i = 0 ; i < rgb_buffer.size(); ++i) {
        frame = rgb_buffer[i];
        output_file.write((char *) frame, sizeof(libfreenect2::Frame));
        size_t data_size = frame->width * frame->height * frame->bytes_per_pixel;
        output_file.write((char *) frame->data, data_size);

        frame = depth_buffer[i];
        output_file.write((char *) frame, sizeof(libfreenect2::Frame));
        data_size = frame->width * frame->height * frame->bytes_per_pixel;
        output_file.write((char *) frame->data, data_size);
    }
    printf("Saved %i frames\n", rgb_buffer.size());
}

void KinectInterface::load_buffer(std::string filename) {
    ifstream input_file(filename.c_str(), ios::binary);
    size_t num_frame;
    input_file.read((char *)&num_frame, sizeof(size_t));
    unsigned char dummy;
    libfreenect2::Frame *frame;
    for (int i = 0 ; i < num_frame; ++i) {
        frame = new libfreenect2::Frame(0,0,0,&dummy);
        input_file.read((char *) frame, sizeof(libfreenect2::Frame));
        size_t data_size = frame->width * frame->height * frame->bytes_per_pixel;
        frame->data = new unsigned char[data_size];
        input_file.read((char *) frame->data, data_size);
        rgb_buffer.push_back(frame);

        frame = new libfreenect2::Frame(0,0,0,&dummy);
        input_file.read((char *) frame, sizeof(libfreenect2::Frame));
        data_size = frame->width * frame->height * frame->bytes_per_pixel;
        frame->data = new unsigned char[data_size];

        input_file.read((char *) frame->data, data_size);
        depth_buffer.push_back(frame);
    }
    printf("Read %i frames\n", rgb_buffer.size());

}

void KinectInterface::clear_buffer() {
    for (int i = 0; i < rgb_buffer.size(); ++i) {
        delete rgb_buffer[i];
        delete depth_buffer[i];
    }
}

KinectInterface::~KinectInterface() {
    stop_device();
    dev->close();
    if (registration)
        delete registration;
    delete listener;
    delete undistorted;
    delete registered;
    delete depth2rgb;
}


void KinectInterface::process_frame_PDFlow(size_t frameID, cv::Mat& out_intensity, cv::Mat& out_depth) {
    process_frame_PDFlow(rgb_buffer[frameID], depth_buffer[frameID], out_intensity, out_depth);
}

void KinectInterface::process_frame_PDFlow(Frame* rgb, Frame* depth, cv::Mat & out_intensity, cv::Mat & out_depth) {
    cv::Mat rgb_mat, depth_mat, depth2rgb_mat;
    //original rgb
    cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgb_mat);
    cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depth_mat);

    //cv::imshow("rgb", rgb_mat);
    //cv::imshow("depth", depth_mat/ 4500.0f);

    registration->apply(rgb, depth, undistorted, registered, true, depth2rgb);
    cv::Mat(depth2rgb->height, depth2rgb->width, CV_32FC1, depth2rgb->data).copyTo(depth2rgb_mat);

    cv::resize(rgb_mat(cv::Rect(240, 0, 1440, 1080)), out_intensity, cv::Size(640, 480));
    cv::resize(depth2rgb_mat(cv::Rect(240, 0, 1440, 1080)), out_depth, cv::Size(640, 480));
    cv::cvtColor(out_intensity, out_intensity, CV_BGR2GRAY);
    out_depth = out_depth*0.001f;
}

void KinectInterface::process_buffer() {
    cv::Mat rgb_mat, depth_mat, depth2rgb_mat;

    for (int i = 0; i < rgb_buffer.size(); ++i) {
        libfreenect2::Frame *rgb = rgb_buffer[i];
        libfreenect2::Frame *depth = depth_buffer[i];

        cv::Mat rgb_rs;
        cv::Mat depth_rs;

        process_frame_PDFlow(rgb, depth, rgb_rs, depth_rs);

        cv::imshow("depth2rgb", depth_rs);
        cv::imshow("rgb", rgb_rs);
        int key = cv::waitKey(1);
    }
}
