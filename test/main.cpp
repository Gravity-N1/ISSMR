// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    //Get each frame
    rs2::frame color_frame = frames.get_color_frame();

    // Creating OpenCV Matrix from a color image
    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

    // Display in a GUI
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", color);

    waitKey(0);

    return 0;
}
