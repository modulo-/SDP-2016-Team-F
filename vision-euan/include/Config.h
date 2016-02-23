#ifndef CONFIG_H
#define CONFIG_H

#include <vector>

#include <opencv2/core/cvstd.hpp>

#include "Pitch.h"

namespace vision {

    class Config {
        public:
            Config(const cv::String&);
            virtual ~Config();

            vision::Pitch getPitch(const size_t) const;

        protected:

        private:
            std::vector<vision::Pitch> pitches;
    };

}
#endif // CONFIG_H
