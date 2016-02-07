#include "Pitch.h"

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace vision {

    Pitch::Pitch(const cv::Mat cm, const cv::Mat dc, const cv::Mat aft, const double a, const cv::Mat bg) : rotationAngle(a) {
        cm.copyTo(cameraMatrix);
        dc.copyTo(distortionCoefficients);
        aft.copyTo(affineTransformation);
        bg.copyTo(backgroundImage);
    }

    Pitch::Pitch(const Pitch& p) : rotationAngle(p.rotationAngle) {
        cameraMatrix = p.cameraMatrix;
        distortionCoefficients = p.distortionCoefficients;
        affineTransformation = p.affineTransformation;
        backgroundImage = p.backgroundImage;
    }

    Pitch::~Pitch() {
        cameraMatrix.release();
        distortionCoefficients.release();
        affineTransformation.release();
        backgroundImage.release();
    }

    cv::UMat Pitch::getCameraMatrix() const {
        return cameraMatrix;
    }

    cv::UMat Pitch::getDistortionCoefficients() const {
        return distortionCoefficients;
    }

    cv::UMat Pitch::getAffineTransformation() const {
        return affineTransformation;
    }

    cv::UMat Pitch::getRotationTransformation(cv::Size frameSize) const {
        cv::Point2f center(frameSize.width / 2, frameSize.height / 2);
        cv::UMat ret;
        cv::getRotationMatrix2D(center, rotationAngle, 1).copyTo(ret);
        return ret;
    }

    cv::UMat Pitch::getBackgroundImage() const {
        return backgroundImage;
    }
}
