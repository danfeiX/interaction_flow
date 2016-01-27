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

    PD_flow_opencv sceneflow(240);

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
        sceneflow.initializeCUDA(640, 480);
        Mat i1, d1, i2, d2;

        for (int i = 0; i < kinect.num_frames()-1; ++i) {
            kinect.process_frame_PDFlow(i, i1, d1);
            kinect.process_frame_PDFlow(i+1, i2, d2);
            sceneflow.loadRGBDFrames(i1, d1, i2, d2);
            sceneflow.solveSceneFlowGPU();
            cv::imshow("intensity", i1);
            cv::imshow("depth", d1);
            sceneflow.showAndSaveResults();
            if (protonect_shutdown)
                break;
        }

        sceneflow.freeGPUMemory();
    }

    return 0;
}