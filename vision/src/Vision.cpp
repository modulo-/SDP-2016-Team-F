
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <iostream>

cv::Mat frame;
cv::Mat proc;

int threshold_value = 0;
int threshold_type = 3;
int framenumber = 0;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

const char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
const char* trackbar_value = "Value";

int main(int argc, char** argv) {
  cv::VideoCapture cam(0);
  //cv::VideoCapture cam("test.avi");

  if(!cam.isOpened()) {
    std::cout << "Cannot open camera feed" << std::endl;
    return 1;
  }

  /*std::cout << "Width: " << cam.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
  std::cout << "Height: " << cam.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
  std::cout << "FPS: " << cam.get(cv::CAP_PROP_FPS) << std::endl;
  std::cout << "Brightness: " << cam.get(cv::CAP_PROP_BRIGHTNESS) << std::endl;
  std::cout << "Contrast: " << cam.get(cv::CAP_PROP_CONTRAST) << std::endl;
  std::cout << "Saturation: " << cam.get(cv::CAP_PROP_SATURATION) << std::endl;
  std::cout << "Hue: " << cam.get(cv::CAP_PROP_HUE) << std::endl;*/


  cv::namedWindow("Camera");
  /*cv::namedWindow("Proc");

  cv::createTrackbar("Frame", "Proc", &framenumber, cam.get(cv::CAP_PROP_FRAME_COUNT)-1);
  cv::createTrackbar( trackbar_type,
                "Proc", &threshold_type,
                max_type);
  cv::createTrackbar( trackbar_value,
                  "Proc", &threshold_value,
                  max_value);*/

  cv::VideoWriter writer("test4.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), cam.get(cv::CAP_PROP_FPS), cv::Size(cam.get(cv::CAP_PROP_FRAME_WIDTH), cam.get(cv::CAP_PROP_FRAME_HEIGHT)));

  if(!writer.isOpened()) {
    std::cout << "Failed to open video writer" << std::endl;
    return 1;
  }


  while(cam.read(frame)) {
    writer.write(frame);
    //cv::cvtColor(frame, proc, cv::COLOR_BGR2HSV);
    /*cv::GaussianBlur(proc, proc, cv::Size(7,7), 1.5, 1.5);
    cv::Canny(proc, proc, 0, 30, 3);*/
    //cv::threshold( proc, proc, threshold_value, max_BINARY_value,threshold_type );
    cv::imshow("Camera", frame);
    //cv::imshow( "Proc", proc );

    //cam.set(cv::CAP_PROP_POS_FRAMES, framenumber);

    //cv::imshow("Proc", proc);
    //if(cv::waitKey(1000/cam.get(cv::CAP_PROP_FPS)) != -1) {
    if(cv::waitKey(1) != -1) {
      break;
    }
  }

  return 0;
}
