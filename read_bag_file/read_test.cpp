#include <librealsense2/rs.hpp> 
#include <opencv2/opencv.hpp>   
#include <iostream>
using namespace rs2;
using namespace cv;
using namespace std;

int main(int argc, char * argv[]) try
{ 
    cout << "播放test.bag的color與著色的depth影格。\n";

    colorizer color_map(6);
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
    config cfg;
    cfg.enable_device_from_file("test2.bag");
    pipe.start(cfg);
      
    while(1)
    {
        frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        
        frame depth = data.get_depth_frame().apply_filter(color_map);
        frame color = data.get_color_frame();

        int index = color.get_frame_number();
        cout << "frame_number=" << index << endl;

        //color與depth的h,w值可能不一樣
        const int w = depth.as<video_frame>().get_width();
        const int h = depth.as<video_frame>().get_height();
        const int w2 = color.as<video_frame>().get_width();
        const int h2 = color.as<video_frame>().get_height();
        
        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat depth_im(h, w, CV_8UC3, (void*)depth.get_data());

        Mat color_im(h2, w2, CV_8UC3, (void*)color.get_data());
        cvtColor(color_im, color_im, COLOR_RGB2BGR);

        // Update the window with new data
        imshow("depth", depth_im);
        imshow("color", color_im);
        if(waitKey(1)>0) break;
    }
    destroyAllWindows();
    pipe.stop();
    system("Pause");
    return 0;
}
catch (const rs2::error & e)
{
    cerr << "RealSense error calling " << e.get_failed_function() << "(" 
        << e.get_failed_args() << "):\n    " << e.what() << endl;
    system("Pause");
    return 1;
}
catch (const exception& e)
{
    cerr << e.what() << endl;
    system("Pause");
    return 1;
}



