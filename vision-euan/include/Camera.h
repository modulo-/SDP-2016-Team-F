#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core/cvstd.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core/mat.hpp>

#include "Pitch.h"

namespace vision {

    class Camera {
        public:
            Camera(const int, const vision::Pitch);
            Camera(const cv::String&, const vision::Pitch);
            virtual ~Camera();

            cv::UMat getBackgroundImage();
            cv::UMat getFrame() const;
            bool update();
        protected:

        private:
            const vision::Pitch pitch;
            cv::VideoCapture camera;
            cv::UMat rawFrame;
            cv::UMat processedFrame;
            cv::UMat working;
    };

}
#endif // CAMERA_H
