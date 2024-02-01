#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include "lighttrack/LightTrack.hpp"
#include "ostrack/OSTrack.hpp"

using namespace std;

// 全局变量定义
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
void LaunchTrack(shared_ptr<T> tracker, int Mode, const string& path){

    cv::VideoCapture cap;
    string display_name = "Track";
    if (Mode == 0 || Mode == 1) {
        cv::namedWindow(display_name, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(display_name, mouse_callback);

        cv::Mat frame;
        cap.open(path);
        cap >> frame;
        cv::imshow(display_name, frame);
        cout << "----------Read Success!!!----------" << endl;
        cv::Mat img;
        while (true) {
            bool ret = cap.read(img);
            if (!ret) {
                cout << "----------Read failed!!!----------" << endl;
                return;
            }

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
        cap.release();
        cv::destroyAllWindows();
    }
    
    else {
        printf("Mode错误，0：视频文件；1：摄像头");
        return;
    }
}


int main(int argc, char* argv[]){
    if (argc != 3){
        fprintf(stderr, "usage: %s [mode] [path]. \n For video, mode=0, path=/xxx/xxx/*.mp4; \n For webcam mode=1, path is cam id.", argv[0]);
        return -1;
    }
    // Mode=0: 视频文件   Mode=1: 摄像头
    int Mode = atoi(argv[1]);
    string path = argv[2];

    // int Mode = 0;
    // string path = "/path/to/video.mp4";

    string z_path = "lighttrack-z.trt";
    string x_path = "lighttrack-x-head.trt";
    string engine_path = "ostrack-256.trt";

    auto tracker = LightTrack::create_tracker(z_path, x_path);
//    auto tracker = OSTrack::create_tracker(engine_path);
    if(tracker == nullptr){
        printf("tracker is nullptr.\n");
        return -1;
    }

    LaunchTrack(tracker, Mode, path);

    return 0;
}
