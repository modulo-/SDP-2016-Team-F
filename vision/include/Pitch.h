#ifndef PITCH_H
#define PITCH_H

#include <opencv2/core/mat.hpp>

namespace vision {

    class Pitch {
        public:
            Pitch(const cv::Mat, const cv::Mat, const cv::Mat, const double, const cv::Mat);
            Pitch(const Pitch&);
            virtual ~Pitch();

            cv::UMat getCameraMatrix() const;
            cv::UMat getDistortionCoefficients() const;
            cv::UMat getAffineTransformation() const;
            cv::UMat getRotationTransformation(cv::Size) const;
            cv::UMat getBackgroundImage() const;
        protected:

        private:
            cv::UMat cameraMatrix;
            cv::UMat distortionCoefficients;
            cv::UMat affineTransformation;
            const double rotationAngle;
            cv::UMat backgroundImage;
    };

}
#endif // PITCH_H
