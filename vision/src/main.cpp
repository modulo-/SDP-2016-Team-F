#include <iostream>
#include <cmath>
#include <tuple>
#include <array>
#include <algorithm>
#include <fstream>
#include <sstream>

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

    if(argc < 2) {
        std::cerr << "Named of named pipe required" << std::endl;
        return 1;
    }
    std::ofstream namedPipe(argv[1], std::ofstream::out);

    vision::Config config("config/config.xml");
    vision::Camera camera(0, config.getPitch(0));
    //vision::Camera camera("/home/euan/Downloads/pitch (1).avi", config.getPitch(0));

    cv::Point2f lastBallPos;
    bool seenBall = false;

    bool seenYellowGreen = false;
    cv::Point2f lastYGPos;
    double lastYGO = 0;
    bool seenYellowPink = false;
    cv::Point2f lastYPPos;
    double lastYPO = 0;
    bool seenBlueGreen = false;
    cv::Point2f lastBGPos;
    double lastBGO = 0;
    bool seenBluePink = false;
    cv::Point2f lastBPPos;
    double lastBPO = 0;

    // Main processing loop
    // Grabs, decodes and stores a frame from capture each iteration
    while(camera.update()) {
        cv::UMat frame = camera.getFrame();

        std::vector<std::vector<struct ColouredCircle> > circles = findColouredCirclesInFrame(frame, camera.getBackgroundImage());

        cv::Point2f ball;
        double ballSize = 0;
        bool ballFound = false;
        for(size_t i = 0; i < circles[0].size(); i++) {
            ballFound = true;
            if(circles[0][i].radius > ballSize) {
                ball = circles[0][i].center;
                ballSize = circles[0][i].radius;
            }
        }

        if(seenBall && euclidianDistance(ball, lastBallPos) > 50) {
            ball = lastBallPos;
        }

        std::vector<struct ColouredCircle> pinkAndGreen = circles[3];
        pinkAndGreen.insert(pinkAndGreen.end(), circles[4].begin(), circles[4].end());

        bool ygFound = false;
        bool ypFound = false;
        bool bgFound = false;
        bool bpFound = false;

        std::vector<struct Robot> robots;
        for(size_t blue = 0; blue < circles[1].size(); blue++) {
            sort(pinkAndGreen.begin(), pinkAndGreen.end(), [circles, blue](struct ColouredCircle x, struct ColouredCircle y) {
                double distX = euclidianDistance(circles[1][blue].center, x.center);
                double distY = euclidianDistance(circles[1][blue].center, y.center);
                return (distX < distY);
            });

            size_t len = 4;
            if(pinkAndGreen.size() < len) {
                continue;
            }

            std::vector<struct ColouredCircle> markers;
            int numG = 0, numP = 0;
            for(size_t i = 0; i < len; i++) {
                markers.push_back(pinkAndGreen[i]);
                if(pinkAndGreen[i].colour == 3) {
                    numP++;
                } else {
                    numG++;
                }
            }

            bool shouldContinue = true;

            for(size_t i = 0; i < markers.size(); i++) {
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
                r.pos = circles[1][blue].center;

                size_t pindex = 0;
                for(size_t i = 0; i < markers.size(); i++) {
                    if(markers[i].colour == 3) {
                        pindex = i;
                    }
                }

                cv::Point2f botLeftOrigin = markers[pindex].center - r.pos;
                r.orientation = -atan2(botLeftOrigin.x, botLeftOrigin.y) - 0.4;
                r.team = 0;
                r.colour = 0;
                r.markers = markers;

                bgFound = true;

            } else {
                r.pos = circles[1][blue].center;

                size_t pindex = 0;
                for(size_t i = 0; i < markers.size(); i++) {
                    if(markers[i].colour == 4) {
                        pindex = i;
                    }
                }

                cv::Point2f botLeftOrigin = markers[pindex].center - r.pos;
                cv::Point2f rot(
                    float(botLeftOrigin.x * cos(2.35619) - (botLeftOrigin.y * sin(2.35619))),
                    float(botLeftOrigin.x * sin(2.35619) + (botLeftOrigin.y * cos(2.35619)))
                );

                cv::Point2f mid2Center = rot;
                cv::Point2f unit(0, 1);
                double cosTheta = std::abs((mid2Center.dot(unit)) / (sqrt(mid2Center.dot(mid2Center)) * sqrt(unit.dot(unit))));

                r.orientation = acos(cosTheta);
                r.team = 0;
                r.colour = 1;
                r.markers = markers;

                bpFound = true;

            }
            robots.push_back(r);
        }

        for(size_t yellow = 0; yellow < circles[2].size(); yellow++) {
            sort(pinkAndGreen.begin(), pinkAndGreen.end(), [circles, yellow](struct ColouredCircle x, struct ColouredCircle y) {
                double distX = euclidianDistance(circles[2][yellow].center, x.center);
                double distY = euclidianDistance(circles[2][yellow].center, y.center);
                return (distX < distY);
            });

            size_t len = 4;
            if(pinkAndGreen.size() < len) {
                continue;
            }

            std::vector<struct ColouredCircle> markers;
            int numG = 0, numP = 0;
            for(size_t i = 0; i < len; i++) {
                markers.push_back(pinkAndGreen[i]);
                if(pinkAndGreen[i].colour == 3) {
                    numP++;
                } else {
                    numG++;
                }
            }

            bool shouldContinue = true;

            for(size_t i = 0; i < markers.size(); i++) {
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
                r.pos = circles[2][yellow].center;

                size_t pindex = 0;
                for(size_t i = 0; i < markers.size(); i++) {
                    if(markers[i].colour == 3) {
                        pindex = i;
                    }
                }

                cv::Point2f botLeftOrigin = markers[pindex].center - r.pos;
                cv::Point2f rot(
                    float(botLeftOrigin.x * cos(2.35619) - (botLeftOrigin.y * sin(2.35619))),
                    float(botLeftOrigin.x * sin(2.35619) + (botLeftOrigin.y * cos(2.35619)))
                );

                cv::Point2f mid2Center = rot;
                cv::Point2f unit(0, 1);
                double cosTheta = std::abs((mid2Center.dot(unit)) / (sqrt(mid2Center.dot(mid2Center)) * sqrt(unit.dot(unit))));

                r.orientation = acos(cosTheta);
                r.team = 1;
                r.colour = 0;
                r.markers = markers;

                ygFound = true;

            } else {
                r.pos = circles[2][yellow].center;

                size_t pindex = 0;
                for(size_t i = 0; i < markers.size(); i++) {
                    if(markers[i].colour == 4) {
                        pindex = i;
                    }
                }

                cv::Point2f botLeftOrigin = markers[pindex].center - r.pos;
                cv::Point2f rot(
                    float(botLeftOrigin.x * cos(2.35619) - (botLeftOrigin.y * sin(2.35619))),
                    float(botLeftOrigin.x * sin(2.35619) + (botLeftOrigin.y * cos(2.35619)))
                );

                cv::Point2f mid2Center = rot;
                cv::Point2f unit(0, 1);
                double cosTheta = std::abs((mid2Center.dot(unit)) / (sqrt(mid2Center.dot(mid2Center)) * sqrt(unit.dot(unit))));

                r.orientation = acos(cosTheta);
                r.team = 1;
                r.colour = 1;
                r.markers = markers;

                ypFound = true;

            }
            robots.push_back(r);
        }

        std::vector<std::string> jsonKeyObjects;

        if(ballFound) {
            lastBallPos = ball;
            seenBall = true;
        } else if(seenBall) {
            ball = lastBallPos;
            seenBall = false;
            cv::circle(frame, ball, 10, cv::Scalar(0, 0, 255), 3);
        }

        if(ballFound) {
            cv::circle(frame, ball, 10, cv::Scalar(0, 0, 255), 3);
            std::stringstream jsonBall;
            jsonBall << "\"b\":{\"x\":" << ball.x << ",\"y\":" << ball.y << "}";
            jsonKeyObjects.push_back(jsonBall.str());
        }

        for(size_t i = 0; i < robots.size(); i++) {
            if(robots[i].team == 0 && robots[i].colour == 0) {
                bgFound = true;
                lastBGPos = robots[i].pos;
                lastBGO = robots[i].orientation;
            } else if(robots[i].team == 0 && robots[i].colour == 1) {
                bpFound = true;
                lastBPPos = robots[i].pos;
                lastBPO = robots[i].orientation;
            } else if(robots[i].team == 1 && robots[i].colour == 0) {
                ygFound = true;
                lastYGPos = robots[i].pos;
                lastYGO = robots[i].orientation;
            } else if(robots[i].team == 1 && robots[i].colour == 1) {
                ypFound = true;
                lastYPPos = robots[i].pos;
                lastYPO = robots[i].orientation;
            }
        }

        if(bgFound) {
            seenBlueGreen = true;
        } else if(seenBlueGreen) {
            seenBlueGreen = false;
            cv::circle(frame, lastBGPos, 10,  cv::Scalar(255, 0, 0), 3);
            cv::circle(frame, lastBGPos, 50,  cv::Scalar(0, 255, 0), 3);
            cv::Point2f newPoint(
                float(0 * cos(lastBGO) - (-20 * sin(lastBGO))),
                float(0 * sin(lastBGO) + (-20 * cos(lastBGO)))
            );
            cv::line(frame, lastBGPos, lastBGPos + newPoint, cv::Scalar(0, 255, 0), 2);
        }

        if(bpFound) {
            seenBluePink = true;
        } else if(seenBluePink) {
            seenBluePink = false;
            cv::circle(frame, lastBPPos, 10,  cv::Scalar(255, 0, 0), 3);
            cv::circle(frame, lastBPPos, 50,  cv::Scalar(255, 0, 255), 3);
            cv::Point2f newPoint(
                float(0 * cos(lastBPO) - (-20 * sin(lastBPO))),
                float(0 * sin(lastBPO) + (-20 * cos(lastBPO)))
            );
            cv::line(frame, lastBPPos, lastBPPos + newPoint, cv::Scalar(0, 255, 0), 2);
        }

        if(ygFound) {
            seenYellowGreen = true;
        } else if(seenYellowGreen) {
            seenYellowGreen = false;
            cv::circle(frame, lastYGPos, 10,  cv::Scalar(0, 255, 255), 3);
            cv::circle(frame, lastYGPos, 50,  cv::Scalar(0, 255, 0), 3);
            cv::Point2f newPoint(
                float(0 * cos(lastYGO) - (-20 * sin(lastYGO))),
                float(0 * sin(lastYGO) + (-20 * cos(lastYGO)))
            );
            cv::line(frame, lastYGPos, lastYGPos + newPoint, cv::Scalar(0, 255, 0), 2);
        }

        if(ypFound) {
            seenYellowPink = true;
        } else if(seenYellowPink) {
            seenYellowPink = false;
            cv::circle(frame, lastYPPos, 10,  cv::Scalar(0, 255, 255), 3);
            cv::circle(frame, lastYPPos, 50,  cv::Scalar(255, 0, 255), 3);
            cv::Point2f newPoint(
                float(0 * cos(lastYPO) - (-20 * sin(lastYPO))),
                float(0 * sin(lastYPO) + (-20 * cos(lastYPO)))
            );
            cv::line(frame, lastYPPos, lastYPPos + newPoint, cv::Scalar(0, 255, 0), 2);
        }

        for(size_t i = 0; i < robots.size(); i++) {
            struct Robot r = robots[i];
            cv::circle(frame, r.pos, 10,  (r.team == 0 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 255)), 3);
            cv::circle(frame, r.pos, 50,  (r.colour == 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 255)), 3);
            cv::Point2f newPoint(
                float(0 * cos(r.orientation) - (-20 * sin(r.orientation))),
                float(0 * sin(r.orientation) + (-20 * cos(r.orientation)))
            );
            cv::line(frame, r.pos, r.pos + newPoint, cv::Scalar(0, 255, 0), 2);

            std::stringstream jsonRobot;

            jsonRobot << '"';
            if(r.team == 0) {
                jsonRobot << 'b';
            } else {
                jsonRobot << 'y';
            }
            if(r.colour == 0) {
                jsonRobot << 'g';
            } else {
                jsonRobot << 'p';
            }
            jsonRobot << "\":{\"x\":" << r.pos.x << ",\"y\":" << r.pos.y << ",\"f\":" << r.orientation * 180 / M_PI << "}";
            jsonKeyObjects.push_back(jsonRobot.str());
        }

        namedPipe << '{';

        for(size_t j = 0; j < jsonKeyObjects.size(); j++) {
            namedPipe << jsonKeyObjects[j];
            if(j != jsonKeyObjects.size() - 1) {
                namedPipe << ',';
            }
        }

        namedPipe << "}\n";
        std::flush(namedPipe);

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

    cv::inRange(processed, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), masks[0]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(76, 90, 90), cv::Scalar(146, 255, 255), masks[1]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(24, 200, 200), cv::Scalar(44, 255, 255), masks[2]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(165, 90, 90), cv::Scalar(180, 255, 255), masks[3]); // HEURISTIC: range of HSV values
    cv::inRange(processed, cv::Scalar(50, 200, 200), cv::Scalar(70, 255, 255), masks[4]); // HEURISTIC: range of HSV values

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
        for(size_t j = 0; j < c.size(); j++) {
            c[j].colour = int(i);
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
