#include <string>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
//#include "cv-helpers.hpp"    // 不用

using namespace std;
using namespace cv;
using namespace rs2;

//函數cv::getStructuringElement形態學侵蝕、膨脹(erode、dilate)
auto gen_element (int erosion_size)
{
    return getStructuringElement(MORPH_RECT,
        Size(erosion_size + 1, erosion_size + 1),
        Point(erosion_size, erosion_size));
};

Mat frame_to_mat(const frame& f) {
    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();
    if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        Mat color = Mat(h, w, CV_8UC3, (void*)f.get_data());
        cvtColor(color, color, COLOR_RGB2BGR);
        return color;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(h,w , CV_16UC1, (void*)f.get_data());
    }
    else
        throw runtime_error("可能其他格式或者就是錯了");
}

int main(int argc, char * argv[]) try
{
    // Define colorizer and align processing-blocks
    colorizer colorize;
    //鏡頭有視角差，透過align_to將depth影像向color影像「對齊」
    rs2::align align_to(RS2_STREAM_COLOR);

    // Start the camera
    pipeline pipe;
    pipe.start();
    cout << "開始depth影像、color影像， intel realsense的語法是用pipeline class\n"
         << "openCV有實做Grab-Cut演算的函數，用到有深度資訊，效果更好，修改demo程式\n"
         << "Grab-Cut演算運算量大，延遲情況顯著\n";
    cout << "Grab-Cut建議參閱https://docs.opencv.org/master/dd/dfc/tutorial_js_grabcut.html\n";
    const string window_name = "去背的圖";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    const int erosion_size = 3;
    auto erode_less = gen_element(erosion_size);
    auto erode_more = gen_element(erosion_size * 2);

    // 此Lambda函數處理grayscale 圖
    //用形態學侵蝕、膨脹(erode、dilate)去掉小洞、對白色區侵蝕
    auto create_mask_from_depth = [&](Mat& depth, int thresh, ThresholdTypes type)
    {
        threshold(depth, depth, thresh, 255, type);
        dilate(depth, depth, erode_less);
        erode(depth, depth, erode_more);
    };

    // Skips some frames to allow for auto-exposure stabilization
    for (int i = 0; i < 10; i++) pipe.wait_for_frames();

    int index = 0;
    while (1) 
    {
        frameset data = pipe.wait_for_frames();
       
        //對齊
        frameset aligned_set = align_to.process(data);
        frame depth = aligned_set.get_depth_frame();
        //frame_to_mat定義在"cv-helpers.hpp"
        Mat color_mat = frame_to_mat(aligned_set.get_color_frame());

        // depth圖用colorizer著色成黑白圖，近為白，遠為黑
        colorize.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        frame bw_depth = depth.apply_filter(colorize);

        // Generate "near" mask image:
        Mat near = frame_to_mat(bw_depth);
        cvtColor(near, near, COLOR_BGR2GRAY);
        // Take just values within range [180-255]
        // These will roughly correspond to near objects due to histogram equalization
        create_mask_from_depth(near, 180, THRESH_BINARY);

        // Generate "far" mask image:
        auto far = frame_to_mat(bw_depth);
        cvtColor(far, far, COLOR_BGR2GRAY);
        far.setTo(255, far == 0); // Note: 0 value does not indicate pixel near the camera, and requires special attention 
        create_mask_from_depth(far, 100, THRESH_BINARY_INV);

        // GrabCut algorithm needs a mask with every pixel marked as either:
        // BGD, FGB, PR_BGD, PR_FGB
        Mat mask;
        mask.create(near.size(), CV_8UC1); 
        mask.setTo(Scalar::all(GC_BGD)); // Set "background" as default guess
        mask.setTo(GC_PR_BGD, far == 0); // Relax this to "probably background" for pixels outside "far" region
        mask.setTo(GC_FGD, near == 255); // Set pixels within the "near" region to "foreground"

        // Run Grab-Cut algorithm:
        Mat bgModel, fgModel; 
        //用 3次迭代
        grabCut(color_mat, mask, Rect(), bgModel, fgModel, 3, GC_INIT_WITH_MASK);

        // Extract foreground pixels based on refined mask from the algorithm
        Mat3b foreground = Mat3b::zeros(color_mat.rows, color_mat.cols);
        color_mat.copyTo(foreground, (mask == GC_FGD) | (mask == GC_PR_FGD));
        imshow(window_name, foreground);
        if (waitKey(1)=='z')
            break;
        else if (waitKey(1000/60) == '+') {//我們增加擷圖功能
            string savefile = to_string(index) + ".png";
            imwrite(savefile, foreground);
            cout << "擷圖至" + savefile << endl;
            index++;
        }
    }
    system("Pause");
    return 0;
}
catch (const rs2::error & e)
{
    cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " 
        << e.what() << endl;
    system("Pause");
    return 1;
}
catch (const exception& e)
{
    cerr << e.what() << endl;
    system("Pause");
    return 1;
}



