#include "Config.h"

#include <exception>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "Pitch.h"

namespace vision {

    //TODO: add loading config from file
    Config::Config(const cv::String&/* filename*/) {
        cv::Point2f src303[3] = {
            cv::Point2f(320, 240),
            cv::Point2f(310, 465),
            cv::Point2f(600, 245)
        };
        cv::Point2f dest303[3] = {
            cv::Point2f(320, 240),
            cv::Point2f(320, 480),
            cv::Point2f(635, 240)
        };

        cv::Point2f src304[3] = {
            cv::Point2f(310, 240),
            cv::Point2f(590, 255),
            cv::Point2f(300, 455)
        };
        cv::Point2f dest304[3] = {
            cv::Point2f(320, 240),
            cv::Point2f(630, 240),
            cv::Point2f(320, 470)
        };

        cv::Mat bg303 = cv::imread("config/pitch303-background.png");
        cv::Mat bg304 = cv::imread("config/pitch304-background.png");

        if(bg303.empty()) {
            throw std::runtime_error("Failed to open pitch303.png");
        }
        if(bg304.empty()) {
            throw std::runtime_error("Failed to open pitch304.png");
        }

        vision::Pitch pitch303(
            (cv::Mat_<double>(3, 3) << 1.2e3, 0, 3.1e2, 0, 1.2e3, 2.3e2, 0, 0, 1),
            (cv::Mat_<double>(5, 1) << -1, 0, 0, 0, 0),
            cv::getAffineTransform(src303, dest303),
            -1,
            bg303);

        vision::Pitch pitch304(
            (cv::Mat_<double>(3, 3) << 1.2e3, 0, 3.1e2, 0, 1.2e3, 2.3e2, 0, 0, 1),
            (cv::Mat_<double>(5, 1) << -1, 0, 0, 0, 0),
            cv::getAffineTransform(src304, dest304),
            0,
            bg304);

        pitches.push_back(pitch303);
        pitches.push_back(pitch304);
    }

    Config::~Config() {}

    vision::Pitch Config::getPitch(const size_t index) const {
        return pitches[index];
    }

}
