
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// Video macros for debugging
#define USE_VIDEO
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
    std::vector<std::vector<cv::Point> > redContours;
    std::vector<std::vector<cv::Point> > blueContours;
    std::vector<std::vector<cv::Point> > yellowContours;
    std::vector<std::vector<cv::Point> > pinkContours;
    std::vector<std::vector<cv::Point> > greenContours;
    std::vector<std::vector<cv::Point> > redHull;
    std::vector<std::vector<cv::Point> > blueHull;
    std::vector<std::vector<cv::Point> > yellowHull;
    std::vector<std::vector<cv::Point> > pinkHull;
    std::vector<std::vector<cv::Point> > greenHull;
    cv::UMat circles;

    // Blur before colour convertion to reduce effects of noise
    cv::medianBlur(processed, blur, 9);

    // Convert to HSV colourspace
    cv::cvtColor(blur, hsv, cv::COLOR_BGR2HSV);

    // Create mask for red
    // Note on how to get HSV values (in Python):
    // cv2.cvtColor(numpy.uint8([[[B,G,R]]]), cv2.COLOR_BGR2HSV) returns [H, S, V] for colour in BGR format
    // Lower bound is then [H - 10, 100, 100] (capping at 0)
    // Upper bound is then [H + 10, 255, 255] (capping at 180)
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10,255,255), redMask);

    // Create mask for blue
    cv::inRange(hsv, cv::Scalar(80, 100, 100), cv::Scalar(100,255,255), blueMask);

    // Create mask for yellow
    cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(40,255,255), yellowMask);

    // Create mask for pink
    cv::inRange(hsv, cv::Scalar(160, 100, 100), cv::Scalar(180,255,255), pinkMask);

    // Create mask for green
    cv::inRange(hsv, cv::Scalar(50, 100, 100), cv::Scalar(70,255,255), greenMask);

    // Find the contours in each of the masks
    cv::findContours(redMask, redContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(blueMask, blueContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(yellowMask, yellowContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(pinkMask, pinkContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(greenMask, greenContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // Find convex hulls from contours, removing those that are too small, too big or not actually convex
    for(size_t i = 0; i<redContours.size();++i) {
        std::vector<cv::Point> hull;
        cv::convexHull(redContours[i], hull);
        double area = cv::contourArea(hull);
        if(cv::isContourConvex(hull) && area >= 50 && area <= 150) {
            redHull.push_back(hull);
        }
    }
    for(size_t i = 0; i<blueContours.size();++i) {
        std::vector<cv::Point> hull;
        cv::convexHull(blueContours[i], hull);
        double area = cv::contourArea(hull);
        if(cv::isContourConvex(hull) && area >= 50 && area <= 150) {
            blueHull.push_back(hull);
        }
    }
    for(size_t i = 0; i<yellowContours.size();++i) {
        std::vector<cv::Point> hull;
        cv::convexHull(yellowContours[i], hull);
        double area = cv::contourArea(hull);
        if(cv::isContourConvex(hull) && area >= 50 && area <= 150) {
            yellowHull.push_back(hull);
        }
    }
    for(size_t i = 0; i<pinkContours.size();++i) {
        std::vector<cv::Point> hull;
        cv::convexHull(pinkContours[i], hull);
        double area = cv::contourArea(hull);
        if(cv::isContourConvex(hull) && area >= 50 && area <= 150) {
            pinkHull.push_back(hull);
        }
    }
    for(size_t i = 0; i<greenContours.size();++i) {
        std::vector<cv::Point> hull;
        cv::convexHull(greenContours[i], hull);
        double area = cv::contourArea(hull);
        if(cv::isContourConvex(hull) && area >= 50 && area <= 150) {
            greenHull.push_back(hull);
        }
    }

    // Debug draw hulls
    processed.copyTo(circles);
    cv::drawContours(circles, redHull, -1, cv::Scalar(0,0,255));
    cv::drawContours(circles, blueHull, -1, cv::Scalar(255,0,0));
    cv::drawContours(circles, yellowHull, -1, cv::Scalar(0,255,255));
    cv::drawContours(circles, pinkHull, -1, cv::Scalar(255,0,255));
    cv::drawContours(circles, greenHull, -1, cv::Scalar(0,255,0));

    // Debug shows
    cv::imshow("Red Mask", redMask);
    cv::imshow("Blue Mask", blueMask);
    cv::imshow("Yellow Mask", yellowMask);
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
