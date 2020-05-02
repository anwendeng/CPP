#include <librealsense2/rs.hpp> 
#include <opencv2/opencv.hpp>   

using namespace rs2;
using namespace cv;
using namespace std;

int main(int argc, char * argv[]) try
{ 
    cout << "同時顯示color與著色的depth影格，未對齊修正，鏡頭視角差明顯。\n";

    colorizer color_map(9);
    /*著色的代碼如下
     0 - Jet    
     1 - Classic 
     2 - WhiteToBlack
     3 - BlackToWhite 
     4 - Bio
     5 - Cold
     6 - Warm
     7 - Quantized
     8 - Pattern
     9 - Hue
    */
    pipeline pipe;
    pipe.start();

    string window_name = "著色的深度圖", win_color = "BGR圖";
    namedWindow(window_name, WINDOW_AUTOSIZE);
    namedWindow(win_color,1);

    while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        frame depth = data.get_depth_frame().apply_filter(color_map);
        frame color = data.get_color_frame();

        int index = color.get_frame_number();

        // Query frame size (width and height)
        const int w = depth.as<video_frame>().get_width();
        const int h = depth.as<video_frame>().get_height();
        const int w2 = color.as<video_frame>().get_width();
        const int h2 = color.as<video_frame>().get_height();
        
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat depth_im(h, w, CV_8UC3, (void*)depth.get_data());

        Mat color_im(h2, w2, CV_8UC3, (void*)color.get_data());
        cvtColor(color_im, color_im, COLOR_RGB2BGR);

        // Update the window with new data
        imshow(window_name, depth_im);
        imshow(win_color, color_im);

        int k = waitKey(1);
        if ( k== 'z') break;
        else if (k == '+') {
            string color_file = "BGR" + to_string(index) + ".png";
            string depth_file = "depth" + to_string(index) + ".png";
            imwrite(color_file, color_im);
            imwrite(depth_file, depth_im);
            cout << index << "寫進圖檔\n";
        }
    }
    return 0;
}
catch (const rs2::error & e)
{
    cerr << "RealSense error calling " << e.get_failed_function() << "(" 
        << e.get_failed_args() << "):\n    " << e.what() << endl;
    return 1;
}
catch (const exception& e)
{
    cerr << e.what() << endl;
    return 1;
}



