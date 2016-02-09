#include "Camera.h"

#include <exception>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace vision {

    Camera::Camera(const int cameraNumber, const vision::Pitch p) : pitch(p) {
        camera = cv::VideoCapture(cameraNumber);

        if(!camera.isOpened()) {
            throw std::runtime_error("Failed to open camera");
        }

        camera.set(cv::CAP_PROP_BRIGHTNESS, 0.5);
        camera.set(cv::CAP_PROP_CONTRAST, 0.5);
        camera.set(cv::CAP_PROP_SATURATION, 0.5);
        camera.set(cv::CAP_PROP_HUE, 0.5);
    }

    Camera::Camera(const cv::String& filename, const vision::Pitch p) : pitch(p) {
        camera = cv::VideoCapture(filename);

        if(!camera.isOpened()) {
            throw std::runtime_error("Failed to open video file " + filename);
        }
    }

    Camera::~Camera() {
        camera.release();
        rawFrame.release();
        processedFrame.release();
        working.release();
    }

    cv::UMat Camera::getBackgroundImage() {
        cv::UMat distort = pitch.getBackgroundImage();
        cv::UMat background;

        cv::Mat optimalCameraMatrix = cv::getOptimalNewCameraMatrix(pitch.getCameraMatrix(), pitch.getDistortionCoefficients(), distort.size(), 0);
        cv::undistort(distort, background, pitch.getCameraMatrix(), pitch.getDistortionCoefficients(), optimalCameraMatrix);
        cv::warpAffine(background, working, pitch.getAffineTransformation(), background.size());
        cv::warpAffine(working, background, pitch.getRotationTransformation(working.size()), working.size());

        return background;
    }

    cv::UMat Camera::getFrame() const {
        return processedFrame;
    }

    bool Camera::update() {
        bool status = camera.read(rawFrame);

        if(status) {
            cv::Mat optimalCameraMatrix = cv::getOptimalNewCameraMatrix(pitch.getCameraMatrix(), pitch.getDistortionCoefficients(), rawFrame.size(), 0);
            cv::undistort(rawFrame, processedFrame, pitch.getCameraMatrix(), pitch.getDistortionCoefficients(), optimalCameraMatrix);
            cv::warpAffine(processedFrame, working, pitch.getAffineTransformation(), processedFrame.size());
            cv::warpAffine(working, processedFrame, pitch.getRotationTransformation(working.size()), working.size());
        }

        return status;
    }
}
