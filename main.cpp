#include <iostream>
#include "kinect_interface.h"
#include <signal.h>
#include <time.h>
#include "scene_flow.h"

using namespace std;
bool protonect_shutdown = false; // Whether the running application should shut down.
void sigint_handler(int s)
{
    protonect_shutdown = true;

}
using namespace std;
using namespace cv;
int main(int argc, char* argv[]) {


    KinectInterface kinect(Processor::cl);
    signal(SIGINT, sigint_handler);

    if (argc < 2) {
        return 0;
    }
    if (std::string(argv[1]) == "--record") {
        int c = 0;
        clock_t t;
        int f;
        t = clock();
        while (!protonect_shutdown) {
            kinect.capture_frame();
            c++;
            float sec = (clock() - t)/CLOCKS_PER_SEC;        
            if (sec > 10) {
                cout << "Record FPS: " << (float)c/sec<< endl;
                t = clock();
                c = 0;
            }
        }
        kinect.save_buffer(argv[2]);
        kinect.clear_buffer();
    } else if (std::string(argv[1]) == "--process") {
        printf("loading buffer...\n");
        kinect.load_buffer(argv[2]);

        cv::Mat display(960, 1280, CV_8UC3, Scalar(0,0,0));

        PD_flow_opencv sceneflow(240);
        sceneflow.initializeCUDA(640, 480);
        Mat i1, d1, i2, d2, flow, flow_res;
        VideoWriter video("out.avi",CV_FOURCC('M','J','P','G'), 20, Size(1280,960),true);

        for (int i = 0; i < kinect.num_frames()-1; ++i) {
            kinect.process_frame_PDFlow(i, i1, d1);
            kinect.process_frame_PDFlow(i+1, i2, d2);
            sceneflow.loadRGBDFrames(i1, d1, i2, d2);
            sceneflow.solveSceneFlowGPU();
            sceneflow.getResult(flow);
            cv::resize(flow, flow_res, cv::Size(640, 480));
            Mat i1_, d1_;
            cv::cvtColor(i1,i1_,COLOR_GRAY2RGB);
            d1.convertTo(d1_, CV_8UC1);
            cv::cvtColor(d1_*100,d1_,COLOR_GRAY2RGB);
            i1_.copyTo(display(cv::Rect(0, 0, 640, 480)));
            d1_.copyTo(display(cv::Rect(640, 0, 640, 480)));

            flow_res.copyTo(display(cv::Rect(0, 480, 640, 480)));
            video.write(display);
            cv::imshow("display", display);
            cv::waitKey(1);
            if (protonect_shutdown)
                break;
        }

        sceneflow.freeGPUMemory();
    }

    return 0;
}