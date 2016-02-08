#include <iostream>
#include <cmath>
#include <tuple>
#include <array>
#include <algorithm>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>

#include "Camera.h"
#include "Config.h"

// Key code of escape key returned by cv::waitKey
#define ESC_KEY 27

/*
Primary colour (BGR & HSV):
    ball: (12, 12, 219), (0, 241, 219)
    blue marker: (95, 108, 11), (86, 229, 108)
    yellow marker: (0, 145, 127), (34,255,145)
    pink marker: (60, 22, 226), (174, 230, 226)
    green marker: (0, 168, 2), (60,255,168)
*/

// Prototypes
void findCircles(std::vector<std::vector<cv::Point> >, std::vector<struct ColouredCircle>&, double, double);
double euclidianDistance(cv::Point2f, cv::Point2f);
std::vector<std::vector<struct ColouredCircle> > findColouredCirclesInFrame(cv::InputArray, cv::InputArray);

struct ColouredCircle {
  cv::Point2f center;
  double radius;
  int colour;
} ColouredCircle;

struct Robot {
  cv::Point2f pos;
  std::vector<struct ColouredCircle> markers;
  double orientation;
  int team;
  int colour;
};

int main(const int argc, const char* argv[]) {
    // Ensure OpenCV is using optimized code
    cv::setUseOptimized(true);

    vision::Config config("config/config.xml");
    //vision::Camera camera("/afs/inf.ed.ac.uk/user/s13/s1322315/Downloads/test4.avi", config.getPitch(1));
    vision::Camera camera(0, config.getPitch(0));

    // Main processing loop
    // Grabs, decodes and stores a frame from capture each iteration
    while(camera.update()) {
        cv::UMat frame = camera.getFrame();

        std::vector<std::vector<struct ColouredCircle> > circles = findColouredCirclesInFrame(frame, camera.getBackgroundImage());

        // TODO: handle case where there is more than one red circle
        cv::Point2f ball;
        bool ballFound = false;
        if(circles[0].size() > 0) {
            ball = circles[0][0].center;
            ballFound = true;
        }

        std::vector<struct ColouredCircle> pinkAndGreen = circles[3];
        pinkAndGreen.insert(pinkAndGreen.end(), circles[4].begin(), circles[4].end());

        std::vector<struct Robot> robots;
        for(size_t blue=0;blue<circles[1].size();blue++) {
          sort(pinkAndGreen.begin(), pinkAndGreen.end(), [circles, blue](struct ColouredCircle x, struct ColouredCircle y) {
            double distX = euclidianDistance(circles[1][blue].center, x.center);
            double distY = euclidianDistance(circles[1][blue].center, y.center);
            return (distX < distY);
          });

          int len = 4;
          if(pinkAndGreen.size() < len) {
            continue;
          }

          std::vector<struct ColouredCircle> markers;
          int numG = 0, numP = 0;
          for(size_t i=0;i<len;i++) {
            markers.push_back(pinkAndGreen[i]);
            if(pinkAndGreen[i].colour == 3) {
              numP++;
            } else {
              numG++;
            }
          }

          bool shouldContinue = true;

          for(size_t i=0;i<markers.size();i++) {
            double dist = euclidianDistance(circles[1][blue].center, markers[i].center);
            if(dist > 50) {
              shouldContinue = false;
              break;
            }
          }

          if(!shouldContinue) {
            continue;
          }

          struct Robot r;
          if(numP < numG) {
            cv::Point2f farGreen1 = markers[3].center;
            cv::Point2f farGreen2 = markers[4].center;
            cv::Point2f mid = (farGreen1+farGreen2) / 2;
            cv::Point2f mid2Center = mid - r.pos;
            cv::Point2f unit(0,1);
            double cosTheta = abs((mid2Center.dot(unit))/(sqrt(mid2Center.dot(mid2Center)) * sqrt(unit.dot(unit))));

            r.pos = circles[1][blue].center;
            r.orientation = acos(cosTheta);
            r.team = 0;
            r.colour = 0;
            r.markers = markers;
          } else {
            cv::Point2f farPink1 = markers[3].center;
            cv::Point2f farPink2 = markers[4].center;
            cv::Point2f mid = (farPink1+farPink2) / 2;
            cv::Point2f mid2Center = mid - r.pos;
            cv::Point2f unit(0,1);
            double cosTheta = abs((mid2Center.dot(unit))/(sqrt(mid2Center.dot(mid2Center)) * sqrt(unit.dot(unit))));

            r.pos = circles[1][blue].center;
            r.orientation = acos(cosTheta);
            r.team = 0;
            r.colour = 1;
            r.markers = markers;
          }
          robots.push_back(r);
        }

        for(size_t yellow=0;yellow<circles[2].size();yellow++) {
          sort(pinkAndGreen.begin(), pinkAndGreen.end(), [circles, yellow](struct ColouredCircle x, struct ColouredCircle y) {
            double distX = euclidianDistance(circles[2][yellow].center, x.center);
            double distY = euclidianDistance(circles[2][yellow].center, y.center);
            return (distX < distY);
          });

          int len = 4;
          if(pinkAndGreen.size() < len) {
            continue;
          }

          std::vector<struct ColouredCircle> markers;
          int numG = 0, numP = 0;
          for(size_t i=0;i<len;i++) {
            markers.push_back(pinkAndGreen[i]);
            if(pinkAndGreen[i].colour == 3) {
              numP++;
            } else {
              numG++;
            }
          }

          bool shouldContinue = true;

          for(size_t i=0;i<markers.size();i++) {
            double dist = euclidianDistance(circles[2][yellow].center, markers[i].center);
            if(dist > 50) {
              shouldContinue = false;
              break;
            }
          }

          if(!shouldContinue) {
            continue;
          }

          struct Robot r;
          if(numP < numG) {
            cv::Point2f farGreen1 = markers[3].center;
            cv::Point2f farGreen2 = markers[4].center;
            cv::Point2f mid = (farGreen1+farGreen2) / 2;
            cv::Point2f mid2Center = mid - r.pos;
            cv::Point2f unit(0,1);
            double cosTheta = abs((mid2Center.dot(unit))/(sqrt(mid2Center.dot(mid2Center)) * sqrt(unit.dot(unit))));

            r.pos = circles[2][yellow].center;
            r.orientation = acos(cosTheta);
            r.team = 1;
            r.colour = 0;
            r.markers = markers;
          } else {
            cv::Point2f farPink1 = markers[3].center;
            cv::Point2f farPink2 = markers[4].center;
            cv::Point2f mid = (farPink1+farPink2) / 2;
            cv::Point2f mid2Center = mid - r.pos;
            cv::Point2f unit(0,1);
            double cosTheta = abs((mid2Center.dot(unit))/(sqrt(mid2Center.dot(mid2Center)) * sqrt(unit.dot(unit))));

            r.pos = circles[2][yellow].center;
            r.orientation = acos(cosTheta);
            r.team = 1;
            r.colour = 1;
            r.markers = markers;
          }
          robots.push_back(r);
        }

        if(ballFound) {
          std::cout << "Ball: " << ball << std::endl;
          cv::circle(frame, ball, 10, cv::Scalar(0,0,255), 3);
        } else {
          std::cout << "Ball not found" << std::endl;
        }

        for(size_t i=0;i<robots.size();i++) {
          struct Robot r = robots[i];
          std::cout << "Robot: " << std::endl;
          std::cout << "\tTeam: " << (r.team == 0 ? "blue" : "yellow") << std::endl;
          std::cout << "\tColour: " << (r.colour == 0 ? "green" : "pink") << std::endl;
          std::cout << "\tPos: " << r.pos << std::endl;
          std::cout << "\tOrientation: " << r.orientation << std::endl;
          /*cv::circle(frame, r.pos, 10,  (r.team == 0 ? cv::Scalar(255,0,0) : cv::Scalar(0,255,255)), 3);
          cv::circle(frame, r.pos, 50,  (r.colour == 0 ? cv::Scalar(0,255,0) : cv::Scalar(255,0,255)), 3);*/
          /*cv::line(frame, r.pos, r.markers[0].center, cv::Scalar(255,0,0), 2);
          cv::line(frame, r.pos, r.markers[1].center, cv::Scalar(255,0,0), 2);*/
          cv::line(frame, r.pos, r.markers[2].center, cv::Scalar(255,0,0), 2);
          cv::line(frame, r.pos, r.markers[3].center, cv::Scalar(255,0,0), 2);
          cv::Point2f newPoint(r.pos.x * cos(r.orientation) - ((r.pos.y-20) * sin(r.orientation)), r.pos.x * sin(r.orientation) + ((r.pos.y-20) * cos(r.orientation)));
          cv::line(frame, r.pos, newPoint, cv::Scalar(255,0,0), 2);
        }

        std::cout << std::endl;
        //cv::line(frame, cv::Point2f(frame.size().width/2, 0), cv::Point2f(frame.size().width/2, frame.size().height), cv::Scalar(0,255,0));
        //cv::line(frame, cv::Point2f(0, frame.size().height/2), cv::Point2f(frame.size().width, frame.size().height/2), cv::Scalar(0,255,0));

        /*cv::setMouseCallback("Frame", [](int e, int x, int y, int, void* u) {
          if(e ==  cv::EVENT_FLAG_LBUTTON) {
            cv::UMat f = *(cv::UMat*)(u);
            std::cout << f.col(x).row(y).getMat(-1) << std::endl;
            //std::cout << x << ", " << y << std::endl;
          }
        }, &frame);*/

        cv::imshow("Frame", frame);

        // Update windows and check if a key has been pressed
        char key = char(cv::waitKey(1));

        // Check if user wants to quit
        if(key == 'q' || key == 'Q' || key == ESC_KEY) {
            break;
        }
    }
}

std::vector<std::vector<struct ColouredCircle> > findColouredCirclesInFrame(cv::InputArray frame, cv::InputArray background) {
    cv::UMat foreground;
    cv::UMat processed;
    cv::Ptr<cv::BackgroundSubtractorMOG2> bs = cv::createBackgroundSubtractorMOG2(1, 254, false); // HEURISTIC: 254 = parameter for background subtractor
    bs->apply(background, foreground, 1);
    bs->apply(frame, foreground, 0);
    frame.copyTo(processed, foreground);

    cv::medianBlur(processed, processed, 9); // HEURISTIC: 9 = amount of blur applied before colour convertion

    cv::cvtColor(processed, processed, cv::COLOR_BGR2HSV);

    std::array<cv::UMat, 5> masks; // order: ball, blue, yellow, pink, green

    cv::inRange(processed, cv::Scalar(0, 150, 150), cv::Scalar(10, 255, 255), masks[0]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(76, 100, 100), cv::Scalar(96, 255, 255), masks[1]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(24, 100, 100), cv::Scalar(44, 255, 255), masks[2]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(164, 100, 100), cv::Scalar(184, 255, 255), masks[3]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(50, 150, 150), cv::Scalar(70, 255, 255), masks[4]); // HEURISTIC: range of HSV values

    // TODO: potentially can be vectorized
    cv::UMat temp;
    for(size_t i = 0; i < masks.size(); i++) {
        for(size_t j = 0; j < masks.size(); j++) {
            if(j == i) continue;
            cv::bitwise_not(masks[j], temp);
            cv::bitwise_and(masks[i], temp, masks[i]);
        }
    }

    std::array<std::vector<std::vector<cv::Point> >, 5> contours; // order: ball, blue, yellow, pink, green

    // TODO: potentially can be vectorized
    for(size_t i = 0; i < masks.size(); i++) {
        cv::findContours(masks[i], contours[i], cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    }

    std::vector<std::vector<struct ColouredCircle> > circles;

    // TODO: potentially can be vectorized
    for(size_t i = 0; i < contours.size(); i++) {
        std::vector<struct ColouredCircle> c;
        findCircles(contours[i], c, 1, 5); // HEURISTIC: deviation of circle and ellipse center and radius
        for(size_t j=0;j<c.size();j++) {
          c[j].colour = i;
        }
        circles.push_back(c);
    }

    return circles;
}

// Takes contours and finds circles
// Theory: find the circle and ellipse that fit around a contour. If the contour is circular, the circle and ellipse should be similar
// Also, if the contour is circular, the fit circle should be a good approximation of it
void findCircles(std::vector<std::vector<cv::Point> > contours, std::vector<struct ColouredCircle> &circles, double centerEpsilon, double radiusEpsilon) {
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
        struct ColouredCircle cc;
        cc.center = cv::Point2f(center.x, center.y);
        cc.radius = radius;
        circles.push_back(cc);
    }
}

double euclidianDistance(cv::Point2f p1, cv::Point2f p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
