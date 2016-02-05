
#include <cstdio>
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// Video macros for debugging
#define USE_VIDEO
#define VIDEO_FILE "/home/euan/Downloads/test4.avi"
#define START_FRAME 10

// Camera number to use (/dev/video#)
#define CAMERA_NUMBER 0

// Key code of escape key returned by cv::waitKey
#define ESC_KEY 27

// Prototypes
void preprocessFrame(cv::InputArray, cv::OutputArray);
void processFrame(cv::InputArray);
void findCircles(std::vector<std::vector<cv::Point> >, std::vector<cv::Vec3f> &c, double, double);
double euclidianDistance(cv::Point2f, cv::Point2f);

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
    // Ensure OpenCV is using optimized code
    cv::setUseOptimized(true);

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
        int frameNumber = START_FRAME;
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
    int64 start = cv::getTickCount();

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
    std::vector<cv::Vec3f> redCircles;
    std::vector<cv::Vec3f> blueCircles;
    std::vector<cv::Vec3f> yellowCircles;
    std::vector<cv::Vec3f> pinkCircles;
    std::vector<cv::Vec3f> greenCircles;

    // Debug image
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
    cv::inRange(hsv, cv::Scalar(25, 100, 100), cv::Scalar(45,255,255), yellowMask);

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

    // Find circles
    findCircles(redContours, redCircles, 1, 1);
    findCircles(blueContours, blueCircles, 1, 1);
    findCircles(yellowContours, yellowCircles, 1, 1);
    findCircles(pinkContours, pinkCircles, 1, 10);
    findCircles(greenContours, greenCircles, 1, 5);

    // Debug draw circles
    processed.copyTo(circles);
    for(size_t i = 0; i < redCircles.size();i++) {
        cv::Vec3f c = redCircles[i];
        cv::circle(circles, cv::Point2f(c[0], c[1]), cvRound(c[2]), cv::Scalar(0,0,255));
    }
    for(size_t i = 0; i < blueCircles.size();i++) {
        cv::Vec3f c = blueCircles[i];
        cv::circle(circles, cv::Point2f(c[0], c[1]), cvRound(c[2]), cv::Scalar(255,0,0));
    }
    for(size_t i = 0; i < yellowCircles.size();i++) {
        cv::Vec3f c = yellowCircles[i];
        cv::circle(circles, cv::Point2f(c[0], c[1]), cvRound(c[2]), cv::Scalar(0,255,255));
    }
    for(size_t i = 0; i < pinkCircles.size();i++) {
        cv::Vec3f c = pinkCircles[i];
        cv::circle(circles, cv::Point2f(c[0], c[1]), cvRound(c[2]), cv::Scalar(255,0,255));
    }
    for(size_t i = 0; i < greenCircles.size();i++) {
        cv::Vec3f c = greenCircles[i];
        cv::circle(circles, cv::Point2f(c[0], c[1]), cvRound(c[2]), cv::Scalar(0,255,0));
    }

    // Debug shows
    cv::imshow("Red Mask", redMask);
    cv::imshow("Blue Mask", blueMask);
    cv::imshow("Yellow Mask", yellowMask);
    cv::imshow("Pink Mask", pinkMask);
    cv::imshow("Green Mask", greenMask);
    cv::imshow("Circles", circles);

    printf("%f\n", (cv::getTickCount() - start) / cv::getTickFrequency());
}

// Takes contours and finds circles
// Theory: find the circle and ellipse that fit around a contour. If the contour is circular, the circle and ellipse should be similar
// Also, if the contour is circular, the fit circle should be a good approximation of it
void findCircles(std::vector<std::vector<cv::Point> > contours, std::vector<cv::Vec3f> &circles, double centerEpsilon, double radiusEpsilon) {
    // Iterate over each contour
    for(size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> contour = contours[i];

        // Discard is less than five points in contour
        // Both as can't fit ellipse otherwise and is unlikely to be a circle with five points
        if(contour.size() < 5) {
            continue;
        }

        // Find minimum enclosing circle
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        // Fit an ellipse around the contour
        cv::RotatedRect ellipse = cv::fitEllipse(contour);
        cv::Point2f eCenter = ellipse.center;
        cv::Point2f eAxesRadius = ellipse.size;
        // Need to half to as currently axes (diameter)
        eAxesRadius.x = eAxesRadius.x / 2;
        eAxesRadius.y = eAxesRadius.y / 2;

        // Compare the circle and ellipse
        // Uses Euclidian distance. Better alternatives?
        double centerDistance = euclidianDistance(center, eCenter);
        double radiusDistance = euclidianDistance(cv::Point2f(radius, radius), eAxesRadius);

        if(centerDistance > centerEpsilon || radiusDistance > radiusEpsilon) {
            continue;
        }

        // Add the found circle to the output vector
        circles.push_back(cv::Vec3f(center.x, center.y, radius));
    }
}

double euclidianDistance(cv::Point2f p1, cv::Point2f p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
