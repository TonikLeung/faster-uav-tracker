#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include "lighttrack/LightTrack.hpp"
#include "ostrack/OSTrack.hpp"
#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>

using namespace std;

// 全局变量定义
const int Cam_WIDTH = 640;
const int Cam_HEIGHT = 480;
const int Cam_RATE = 60;
float point_in_color_coordinates[3];

cv::Rect bbox;
bool drawing_box = false;
bool tracker_init = false;
bool box_init = false;

void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    switch (event) {
        case cv::EVENT_LBUTTONDOWN:
            drawing_box = true;
            tracker_init = false;
            // 开始一个新的框
            bbox = cv::Rect(x, y, 0, 0);
            break;
        case cv::EVENT_LBUTTONUP:
            // 标准化矩形（确保宽度和高度为正）
            if (bbox.width < 0) {
                bbox.x += bbox.width; 
                bbox.width *= -1;
            }
            if (bbox.height < 0) {
                bbox.y += bbox.height;
                bbox.height *= -1;
            }
            drawing_box = false;
            // 判断矩形框初始化是否成功
            if (bbox.height!=0 && bbox.width!=0) {
                box_init = true;
            }
            break;
        case cv::EVENT_MOUSEMOVE:
            if (drawing_box) {
                // 更新矩形尺寸
                bbox.width = x - bbox.x;
                bbox.height = y - bbox.y;
            }
            break;
    }
}

template<class T>
void LaunchTrack(shared_ptr<T> tracker){
    string display_name = "Track";

    // 初始化相机参数
    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, Cam_WIDTH, Cam_HEIGHT, RS2_FORMAT_Z16, Cam_RATE);
    cfg.enable_stream(RS2_STREAM_COLOR, Cam_WIDTH, Cam_HEIGHT, RS2_FORMAT_BGR8, Cam_RATE);
    rs2::pipeline_profile profile = pipe.start(cfg); 
    rs2::align align_to_color(RS2_STREAM_COLOR);   // 创建深度流和彩色流对齐的实例化对象

    rs2::frameset frameset = pipe.wait_for_frames();
    auto aligned_frameset = align_to_color.process(frameset); // 实际进行流对齐
    // 基于对齐的混合流获取深度流和彩色流,进而获取流对齐后的深度内参
    rs2::video_frame color_stream = aligned_frameset.get_color_frame();
    rs2::depth_frame aligned_depth_stream = aligned_frameset.get_depth_frame();

    Mat image(Size(color_w,color_h),CV_8UC3,(void*)color_stream.get_data(),Mat::AUTO_STEP);

    cv::namedWindow(display_name, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(display_name, mouse_callback);

    cv::Mat frame;
    cap.open(path);
    cap >> frame;

    cv::imshow(display_name, image);
    cout << "----------Read Success!!!----------" << endl;
    while (true) {
        // 获取目标相对无人机的位置
        rs2::frameset frameset = pipe.wait_for_frames();
        auto aligned_frameset = align_to_color.process(frameset); // 实际进行流对齐
        // 基于对齐的混合流获取深度流和彩色流,进而获取流对齐后的深度内参
        rs2::video_frame color_stream = aligned_frameset.get_color_frame();
        rs2::depth_frame aligned_depth_stream = aligned_frameset.get_depth_frame();

        rs2::video_stream_profile depth_stream_profile =                 
        aligned_depth_stream.get_profile().as<rs2::video_stream_profile>();
        // 获取彩色图像宽高
        const int color_w=color_stream.as<rs2::video_frame>().get_width();
        const int color_h=color_stream.as<rs2::video_frame>().get_height();

        Mat img(Size(color_w,color_h),CV_8UC3,(void*)color_stream.get_data(),Mat::AUTO_STEP);

        // 初始化 Tracker
        if (!tracker_init) {
            if (box_init) {
                tracker->init(img, bbox);
                tracker_init = true;
                box_init = false;
            }
            if (drawing_box && bbox.width > 0 && bbox.height > 0) {
                cv::rectangle(img, bbox, cv::Scalar(255, 0, 0), 2);
            }
        }

        // 开始跟踪目标
        else {
            auto start = std::chrono::steady_clock::now();
            bbox = tracker->track(img);
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double time = 1000 * elapsed.count();
            printf("all infer time: %f ms\n", time);
            cv::rectangle(img, bbox, cv::Scalar(0,255,0), 2);

            const auto depth_intrinsics = depth_stream_profile.get_intrinsics(); //获取对齐后的深度内参
            float center_x = (bbox[0] + bbox[2])/2
            float center_y = (bbox[1] + bbox[3])/2

            pixe_center[1] = detect_object.y;
            float pixed_center_depth_value = aligned_depth_stream.get_distance(center_x,center_y);
            rs2_deproject_pixel_to_point(point_in_color_coordinates, &depth_intrinsics, pixe_center, pixed_center_depth_value);
            cout << "X: " << point_in_color_coordinates[0] << "Y: " << point_in_color_coordinates[1] << "Z: " << point_in_color_coordinates[2] << endl;
        }

        cv::putText(img, "Esc to quit, r to reset", cv::Point2i(20, 30), 
        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0), 1);

        cv::imshow(display_name, img);

        int key = cv::waitKey(30);

        // 按 Esc 退出
        if (key == 27) break;
        
        // 按 "r" 或单击鼠标重置边界框
        else if (key == 114 || drawing_box)
        {
            tracker_init = false;
            box_init = false;
        }            
    }
    // cap.release();
    cv::destroyAllWindows();
}


int main(int argc, char* argv[]){

    string z_path = "lighttrack-z.trt";
    string x_path = "lighttrack-x-head.trt";
    string engine_path = "ostrack-256.trt";

    auto tracker = LightTrack::create_tracker(z_path, x_path);
//    auto tracker = OSTrack::create_tracker(engine_path);
    if(tracker == nullptr){
        printf("tracker is nullptr.\n");
        return -1;
    }

    LaunchTrack(tracker);

    return 0;
}
