//
//  main.cpp
//  OpenCV Sandbox
//
//  Created by Euan James Hunter on 30/01/2016.
//  Copyright (c) 2016 EJH. All rights reserved.
//

#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#define RANGE 100

using namespace cv;
using namespace std;

Mat camMat = (Mat_<double>(3,3) << 1.2e3, 0, 3.1e2, 0, 1.2e3, 2.3e2, 0, 0, 1);
Mat distCoef;

int main(int argc, const char * argv[]) {
    VideoCapture vid("/Users/euan/Desktop/test.avi");
    
    if(!vid.isOpened()) {
        cout << "Failed to open video" << endl;
        return 1;
    }
    
    int framenumber = 0;
    int k1 = RANGE, k2 = RANGE, p1 = RANGE, p2 = RANGE, k3 = RANGE;
    
    namedWindow("Video");
    createTrackbar("Frame Number", "Video", &framenumber, vid.get(CAP_PROP_FRAME_COUNT)-1);
    createTrackbar("K1", "Video", &k1, RANGE*2);
    createTrackbar("K2", "Video", &k2, RANGE*2);
    //createTrackbar("P1", "Video", &p1, RANGE*2);
    //createTrackbar("P2", "Video", &p2, RANGE*2);
    createTrackbar("K3", "Video", &k3, RANGE*2);
    
    Mat frame;
    Mat undist;
    
    while(vid.read(frame)) {
        distCoef = (Mat_<double>(5,1) <<
                    (double)(k1-RANGE),
                    (double)(k2-RANGE),
                    (double)(p1-RANGE),
                    (double)(p2-RANGE),
                    (double)(k3-RANGE));
        undistort(frame, undist, camMat, distCoef);
        
        imshow("Video", undist);
        
        if(waitKey(1) != -1) {
            break;
        }
        
        vid.set(CAP_PROP_POS_FRAMES, framenumber);
    }
    
    return 0;
}
