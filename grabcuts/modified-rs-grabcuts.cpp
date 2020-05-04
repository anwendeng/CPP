#include <string>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
//#include "cv-helpers.hpp"    // ����

using namespace std;
using namespace cv;
using namespace rs2;

//���cv::getStructuringElement�κA�ǫI�k�B����(erode�Bdilate)
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
        throw runtime_error("�i���L�榡�Ϊ̴N�O���F");
}

int main(int argc, char * argv[]) try
{
    // Define colorizer and align processing-blocks
    colorizer colorize;
    //���Y�������t�A�z�Lalign_to�Ndepth�v���Vcolor�v���u����v
    rs2::align align_to(RS2_STREAM_COLOR);

    // Start the camera
    pipeline pipe;
    pipe.start();
    cout << "�}�ldepth�v���Bcolor�v���A intel realsense���y�k�O��pipeline class\n"
         << "openCV���갵Grab-Cut�t�⪺��ơA�Ψ즳�`�׸�T�A�ĪG��n�A�ק�demo�{��\n"
         << "Grab-Cut�t��B��q�j�A���𱡪p���\n";
    cout << "Grab-Cut��ĳ�Ѿ\https://docs.opencv.org/master/dd/dfc/tutorial_js_grabcut.html\n";
    const string window_name = "�h�I����";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    const int erosion_size = 3;
    auto erode_less = gen_element(erosion_size);
    auto erode_more = gen_element(erosion_size * 2);

    // ��Lambda��ƳB�zgrayscale ��
    //�ΧκA�ǫI�k�B����(erode�Bdilate)�h���p�}�B��զ�ϫI�k
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
       
        //���
        frameset aligned_set = align_to.process(data);
        frame depth = aligned_set.get_depth_frame();
        //frame_to_mat�w�q�b"cv-helpers.hpp"
        Mat color_mat = frame_to_mat(aligned_set.get_color_frame());

        // depth�ϥ�colorizer�ۦ⦨�¥չϡA�񬰥աA������
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
        //�� 3�����N
        grabCut(color_mat, mask, Rect(), bgModel, fgModel, 3, GC_INIT_WITH_MASK);

        // Extract foreground pixels based on refined mask from the algorithm
        Mat3b foreground = Mat3b::zeros(color_mat.rows, color_mat.cols);
        color_mat.copyTo(foreground, (mask == GC_FGD) | (mask == GC_PR_FGD));
        imshow(window_name, foreground);
        if (waitKey(1)=='z')
            break;
        else if (waitKey(1000/60) == '+') {//�ڭ̼W�[�^�ϥ\��
            string savefile = to_string(index) + ".png";
            imwrite(savefile, foreground);
            cout << "�^�Ϧ�" + savefile << endl;
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



