#include <librealsense2/rs.hpp> 
#include <opencv2/opencv.hpp>   

using namespace rs2;
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    cout << "同時顯示color, depth IR影格，未對齊修正，鏡頭視角差明顯。\n";

    colorizer color_map(5);

    pipeline pipe;
    config cfg;
    int w = 640, h = 480, FPS = 15;//可用的設定請參閱rs-enumerate-devices
    cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, FPS);
    cfg.enable_stream(RS2_STREAM_INFRARED, w, h, RS2_FORMAT_Y8, FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, FPS);
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    frameset data;
    for (int i = 0; i < FPS; i++) data = pipe.wait_for_frames();

    while (1)
    {
        frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        frame color = data.get_color_frame();
        int index = color.get_frame_number();
        frame depth = data.get_depth_frame().apply_filter(color_map);

        frame ir = data.get_infrared_frame();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat color_im(h, w, CV_8UC3, (void*)color.get_data());
        Mat depth_im(h, w, CV_8UC3, (void*)depth.get_data());
        Mat ir_im(h, w, CV_8UC1, (void*)ir.get_data());
        // Apply Histogram Equalization
        equalizeHist(ir_im, ir_im);
        applyColorMap(ir_im, ir_im, COLORMAP_JET);

        // Update the window with new data
        imshow("color", color_im);
        imshow("ir", ir_im);
        imshow("depth", depth_im);
        int k = waitKey(1);
        if (k == 'z') break;
        else if (k == '+') {
            string color_file = "BGR" + to_string(index) + ".png";
            string ir_file = "ir" + to_string(index) + ".png";
            string depth_file = "depth" + to_string(index) + ".png";
            imwrite(color_file, color_im);
            imwrite(ir_file, ir_im);
            imwrite(depth_file, depth_im);
            cout << index << "寫進圖檔\n";
        }
    }
    destroyAllWindows();
    pipe.stop();
    return 0;
}