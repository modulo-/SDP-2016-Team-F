
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// Video macros for debugging
//#define USE_VIDEO
#define VIDEO_FILE "/home/euan/Downloads/test4.avi"

// Camera number to use (/dev/video#)
#define CAMERA_NUMBER 0

// Key code of escape key returned by cv::waitKey
#define ESC_KEY 27

// Prototypes
void preprocessFrame(cv::InputArray, cv::OutputArray);
void processFrame(cv::InputArray);
void drawCircles(std::vector<cv::Vec3f>, cv::InputOutputArray, cv::Scalar);

/*
    Pipeline:
        Capture image from camera
        Undistort
        Affine transform
        Convert to HSV
        Blur
        Generate red mask (for ball)
        Generate blue mask (for team marker)
        Generate yellow mask (for team marker)
        Generate pink mask (for robot marker)
        Generate fluorescent green mask (for robot marker)
        Circle detect all masks
        Filter circles to remove too small and too big
        Identify ball and robots from circles
        Translate robot position to pitch space and calculate orientation
*/

int main(const int argc, const char* argv[]) {
    // Open camera (or video file)
    #ifdef USE_VIDEO
        cv::VideoCapture capture(VIDEO_FILE);
    #else
        cv::VideoCapture capture(CAMERA_NUMBER);
    #endif // USE_VIDEO

    // Check capture opened properly
    if(!capture.isOpened()) {
        #ifdef USE_VIDEO
            fprintf(stderr, "Failed to open video file %s\n", VIDEO_FILE);
        #else
            fprintf(stderr, "Failed to open video stream %d\n", CAMERA_NUMBER);
        #endif // USE_VIDEO

        return EXIT_FAILURE;
    }

    // Create a window to show the raw capture feed
    // Flags: size window to image, user can't resize
    cv::namedWindow("Capture", cv::WINDOW_AUTOSIZE);

    // Debug windows
    cv::namedWindow("Red Mask");
    cv::namedWindow("Blue Mask");
    cv::namedWindow("Yellow Mask");
    cv::namedWindow("Pink Mask");
    cv::namedWindow("Green Mask");
    cv::namedWindow("Circles");

    // If using video, create trackbar to allow scrubing
    #ifdef USE_VIDEO
        int frameNumber = 0;
        cv::createTrackbar("Frame Number", "Capture", &frameNumber, int(capture.get(cv::CAP_PROP_FRAME_COUNT))-1);
    #endif // USE_VIDEO

    // Main processing loop
    // Grabs, decodes and stores a frame from capture each iteration
    cv::UMat frame;
    cv::UMat processed;
    while(capture.read(frame)) {
        // Preprocess the raw captured frame
        preprocessFrame(frame, processed);

        // Main processing
        processFrame(processed);

        // Show the raw frame
        cv::imshow("Capture", frame);

        // Wait for a bit
        // Allows for windows to update and keyboard interaction
        char key = char(cv::waitKey(int(1000 / capture.get(cv::CAP_PROP_FPS))));

        // Check if user wants to quit
        if(key == 'q' || key == 'Q' || key == ESC_KEY) {
            break;
        }

        // If using video, freeze frame
        #ifdef USE_VIDEO
            capture.set(cv::CAP_PROP_POS_FRAMES, frameNumber);
        #endif // USE_VIDEO
    }
}

// Preprocess frame to remove distortion, and correct alignment
void preprocessFrame(cv::InputArray frame, cv::OutputArray processed) {
    // TODO:
    frame.copyTo(processed);
}

// Do main frame processing
void processFrame(cv::InputArray processed) {
    cv::UMat hsv;
    cv::UMat blur;
    cv::UMat redMask;
    cv::UMat blueMask;
    cv::UMat yellowMask;
    cv::UMat pinkMask;
    cv::UMat greenMask;
    cv::UMat circles;
    std::vector<cv::Vec3f> redCicles;
    std::vector<cv::Vec3f> blueCicles;
    std::vector<cv::Vec3f> yellowCicles;
    std::vector<cv::Vec3f> pinkCicles;
    std::vector<cv::Vec3f> greenCicles;

    // Blur before colour convertion to reduce effects of noise
    cv::medianBlur(processed, blur, 9);

    // Convert to HSV colourspace
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

    // Create mask for red
    // Note on how to get HSV values (in Python):
    // cv2.cvtColor(numpy.uint8([[[B,G,R]]]), cv2.COLOR_BGR2HSV) returns [H, S, V] for colour in BGR format
    // Lower bound is then [H - 10, 100, 100] (capping at 0)
    // Upper bound is then [H + 10, 255, 255] (capping at 255)
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10,255,255), redMask);

    // Create mask for blue
    cv::inRange(hsv, cv::Scalar(80, 100, 100), cv::Scalar(100,255,255), blueMask);

    // Create mask for yellow
    cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(40,255,255), yellowMask);

    // Create mask for pink
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180,255,255), pinkMask);

    // Create mask for green
    cv::inRange(hsv, cv::Scalar(50, 100, 100), cv::Scalar(70,255,255), greenMask);

    // Blur masks to make circle detection easier
    cv::GaussianBlur(redMask, redMask, cv::Size(9,9), 2, 2);
    cv::GaussianBlur(blueMask, blueMask, cv::Size(9,9), 2, 2);
    cv::GaussianBlur(yellowMask, yellowMask, cv::Size(9,9), 2, 2);
    cv::GaussianBlur(pinkMask, pinkMask, cv::Size(9,9), 2, 2);
    cv::GaussianBlur(greenMask, greenMask, cv::Size(9,9), 2, 2);

    // Detect circles in each of the masks
    cv::HoughCircles(redMask, redCicles, CV_HOUGH_GRADIENT, 1, 10, 50, 20, 0, 0);
    cv::HoughCircles(blueMask, blueCicles, CV_HOUGH_GRADIENT, 1, 10, 50, 20, 0, 0);
    //cv::HoughCircles(yellowMask, yellowCicles, CV_HOUGH_GRADIENT, 1, yellowMask.rows/8, 50, 20, 0, 0);
    cv::HoughCircles(pinkMask, pinkCicles, CV_HOUGH_GRADIENT, 1, 10, 50, 20, 0, 0);
    cv::HoughCircles(greenMask, greenCicles, CV_HOUGH_GRADIENT, 1, 10, 50, 20, 0, 0);

    // Debug draw circles on masks
    processed.copyTo(circles);
    drawCircles(redCicles, circles, cv::Scalar(0,0,255));
    drawCircles(blueCicles, circles, cv::Scalar(255,0,0));
    //drawCircles(yellowCicles, circles, cv::Scalar(0,255,255));
    drawCircles(pinkCicles, circles, cv::Scalar(255,0,255));
    drawCircles(greenCicles, circles, cv::Scalar(0,255,0));

    // Debug shows
    cv::imshow("Red Mask", redMask);
    cv::imshow("Blue Mask", blueMask);
    //cv::imshow("Yellow Mask", yellowMask);
    cv::imshow("Pink Mask", pinkMask);
    cv::imshow("Green Mask", greenMask);
    cv::imshow("Circles", circles);
}

void drawCircles(std::vector<cv::Vec3f> circles, cv::InputOutputArray img, cv::Scalar colour) {
    for(size_t c=0;c<circles.size(); c++) {
        cv::Point center(cvRound(circles[c][0]), cvRound(circles[c][1]));
        int radius = cvRound(circles[c][2]);

        cv::circle(img, center, radius, colour, 5);
    }
}
