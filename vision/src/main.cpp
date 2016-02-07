#include <iostream>
#include <cmath>
#include <tuple>
#include <array>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include "Camera.h"
#include "Config.h"

// Key code of escape key returned by cv::waitKey
#define ESC_KEY 27

// Type for contours. Purely to aid readablility
#define CONTOUR_T std::vector<std::vector<cv::Point> >

/*
Primary colour (BGR & HSV):
    ball: (109, 82, 231), (175, 164, 231)
    blue marker: (238, 214, 147), (98, 98, 238)
    yellow marker: (165, 255, 255), (30,90,255)
    pink marker: (193, 130, 255), (165, 125, 255)
    green marker: (144, 255, 188), (48,111,255)
*/

// Prototypes
void processFrame(cv::InputArray);
void findCircles(CONTOUR_T, std::vector<cv::Vec3f> &c, double, double);
double euclidianDistance(cv::Point2f, cv::Point2f);
std::vector<std::tuple<int, cv::Vec3f> > findColouredCirclesInFrame(cv::InputArray, cv::InputArray);
void drawTupleCircle(std::tuple<int, cv::Vec3f>, cv::InputOutputArray);

int main(const int argc, const char* argv[]) {
    // Ensure OpenCV is using optimized code
    cv::setUseOptimized(true);

    vision::Config config("config/config.xml");
    vision::Camera camera("/home/euan/Downloads/test4.avi", config.getPitch(1));

    // Main processing loop
    // Grabs, decodes and stores a frame from capture each iteration
    while(camera.update()) {
        int64 loopStart = cv::getTickCount();

        cv::UMat frame = camera.getFrame();

        std::vector<std::tuple<int, cv::Vec3f> > circles = findColouredCirclesInFrame(frame, camera.getBackgroundImage());

        for(size_t i = 0; i < circles.size(); i++) {
            drawTupleCircle(circles[i], frame);
        }

        // This is really inefficient, and probably broken

        cv::Point2f ball;
        for(size_t i = 0; i < circles.size(); i++) {
            // TODO: handle case where more than one red circle detected
            if(std::get<0>(circles[i]) == 0) {
                ball.x = std::get<1>(circles[i])[0];
                ball.y = std::get<1>(circles[i])[1];
            }
        }

        // This is also really inflexible. Again, probably broken
        std::vector<cv::Vec3f> robots;
        for(size_t i = 0; i < circles.size(); i++) {
            if(std::get<0>(circles[i]) == 1 && std::get<0>(circles[i]) == 2) {
                std::vector<cv::Point2f> markers;
                for(size_t m = 0; m < circles.size(); m++) {
                    if(circles[i] != circles[m]) {
                        if(markers.size() < 4) {
                            markers.push_back(cv::Point2f(std::get<1>(circles[m][0], std::get<1>(circles[m][1])));
                        } else {
                            double candDist = euclidianDistance(cv::Point2f(std::get<1>(circles[m][0], std::get<1>(circles[m][1]))), cv::Point2f(std::get<1>(circles[i][0], std::get<1>(circles[i][1]));
                            for(size_t c = 0; c < markers.size(); c++) {
                            if(candDist < euclidianDistance(markers[c], cv::Point2f(std::get<1>(circles[i][0], std::get<1>(circles[i][1]))) {
                                markers[c] = cv::Point2f(std::get<1>(circles[m][0], std::get<1>(circles[m][1])));
                                    break;
                                }
                            }
                        }
                    }
                }


            }
        }

        // Show the raw frame
        cv::imshow("Capture", frame);

        // Update windows and check if a key has been pressed
        char key = char(cv::waitKey(1));

        // Check if user wants to quit
        if(key == 'q' || key == 'Q' || key == ESC_KEY) {
            break;
        }

        std::cout << (cv::getTickCount() - loopStart) / cv::getTickFrequency() << std::endl;
    }
}

std::vector<std::tuple<int, cv::Vec3f> > findColouredCirclesInFrame(cv::InputArray frame, cv::InputArray background) {
    cv::UMat foreground;
    cv::UMat processed;
    cv::Ptr<cv::BackgroundSubtractorMOG2> bs = cv::createBackgroundSubtractorMOG2(1, 254, false); // HEURISTIC: 254 = parameter for background subtractor
    bs->apply(background, foreground, 1);
    bs->apply(frame, foreground, 0);
    frame.copyTo(processed, foreground);

    cv::medianBlur(processed, processed, 9); // HEURISTIC: 9 = amount of blur applied before colour convertion

    cv::cvtColor(processed, processed, cv::COLOR_BGR2HSV);

    std::array<cv::UMat, 5> masks; // order: ball, blue, yellow, pink, green

    cv::inRange(processed, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), masks[0]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(88, 100, 100), cv::Scalar(108, 255, 255), masks[1]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(20, 100, 100), cv::Scalar(40, 255, 255), masks[2]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(155, 100, 100), cv::Scalar(175, 255, 255), masks[3]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(38, 100, 100), cv::Scalar(58, 255, 255), masks[4]); // HEURISTIC: range of HSV values

    // TODO: potentially can be vectorized
    cv::UMat temp;
    for(size_t i = 0; i < masks.size(); i++) {
        for(size_t j = 1; j < masks.size(); j++) {
            cv::bitwise_not(masks[j], temp);
            cv::bitwise_and(masks[i], temp, masks[i]);
        }
    }

    std::array<CONTOUR_T, 5> contours; // order: ball, blue, yellow, pink, green

    // TODO: potentially can be vectorized
    for(size_t i = 0; i < masks.size(); i++) {
        cv::findContours(masks[i], contours[i], cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    }

    std::vector<std::tuple<int, cv::Vec3f> > circles;

    // TODO: potentially can be vectorized
    for(size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Vec3f> c;
        findCircles(contours[i], c, 1, 5); // HEURISTIC: deviation of circle and ellipse center and radius
        for(size_t j = 0; j < c.size(); j++) {
            circles.push_back(std::make_tuple(i, c[j]));
        }
    }

    return circles;
}

// Takes contours and finds circles
// Theory: find the circle and ellipse that fit around a contour. If the contour is circular, the circle and ellipse should be similar
// Also, if the contour is circular, the fit circle should be a good approximation of it
void findCircles(CONTOUR_T contours, std::vector<cv::Vec3f> &circles, double centerEpsilon, double radiusEpsilon) {
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

void drawTupleCircle(std::tuple<int, cv::Vec3f> tup, cv::InputOutputArray img) {
    cv::Vec3f c = std::get<1>(tup);
    cv::Scalar colour;
    switch(std::get<0>(tup)) {
        default:
        case 0:
            colour = cv::Scalar(0, 0, 255);
            break;
        case 1:
            colour = cv::Scalar(255, 0, 0);
            break;
        case 2:
            colour = cv::Scalar(0, 255, 255);
            break;
        case 3:
            colour = cv::Scalar(255, 0, 255);
            break;
        case 4:
            colour = cv::Scalar(0, 255, 0);
            break;
    }

    cv::circle(img, cv::Point2f(c[0], c[1]), cvRound(c[2]), colour);
}
