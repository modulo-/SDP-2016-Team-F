#include <iostream>
//#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

#include "Camera.h"
#include "Config.h"

// Key code of escape key returned by cv::waitKey
#define ESC_KEY 27

void subtractBackground(cv::InputArray frame, cv::InputArray background, cv::OutputArray foreground) {
    cv::UMat foregroundMask;
    cv::Ptr<cv::BackgroundSubtractorMOG2> subtractor = cv::createBackgroundSubtractorMOG2(1, 256, false);

    subtractor->apply(background, foregroundMask, 1);
    subtractor->apply(frame, foregroundMask, 0);

    frame.copyTo(foreground, foregroundMask);
}

int main(const int argc, const char* argv[]) {
    // Ensure OpenCV is using optimized code
    cv::setUseOptimized(true);

    /*if(argc < 2) {
        std::cerr << "Named of named pipe required" << std::endl;
        return 1;
    }
    std::ofstream namedPipe(argv[1], std::ofstream::out);*/

    vision::Config config("config/config.xml");
    vision::Camera camera(0, config.getPitch(0));
    cv::VideoWriter write("pitch.avi", cv::VideoWriter::fourcc('M','J','P','G'), 25, cv::Size(640, 480));
    if(!write.isOpened())
        return 1;

    // Main processing loop
    // Grabs, decodes and stores a frame from capture each iteration
    while(camera.update()) {
        cv::UMat frame = camera.getFrame();

        /*cv::UMat foreground;
        subtractBackground(frame, camera.getBackgroundImage(), foreground);

        cv::UMat blur;
        cv::medianBlur(foreground, blur, 9);

        cv::UMat binblur;
        cv::inRange(blur, cv::Scalar(1,1,1), cv::Scalar(255,255,255), binblur);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(binblur, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
        for(size_t i=0;i<contours.size();i++) {
            cv::Rect rect = cv::minAreaRect(contours[i]).boundingRect();
            cv::Mat temp;
            frame.copyTo(temp);
            if(0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= frame.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= frame.rows) {
                cv::UMat roiMat = frame(rect);
                cv::Scalar mean, stddev;
                cv::meanStdDev(roiMat, mean, stddev);
                std::cout << "Mean: " << mean << std::endl;
                std::cout << "SD: " << stddev << std::endl;
                cv::rectangle(temp, rect, mean);
            }
            temp.copyTo(frame);
        }
        std::cout << std::endl;*/

        cv::imshow("Frame", frame);
        write.write(frame.getMat(-1));
        // Update windows and check if a key has been pressed
        char key = char(cv::waitKey(1));

        // Check if user wants to quit
        if(key == 'q' || key == 'Q' || key == ESC_KEY) {
            break;
        }
    }
}
