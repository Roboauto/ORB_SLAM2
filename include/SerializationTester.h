/**
 * Created by Matous Kolarik on 9/23/16.
 */

#ifndef ORB_SLAM2_SERIALIZATIONTESTER_H
#define ORB_SLAM2_SERIALIZATIONTESTER_H


#include <opencv-3.1.0-dev/opencv2/core/mat.hpp>

class SerializationTester {
public:
    static constexpr double MAX_REAL_NUM_DIFF = 0.001;

    static bool realNumberEquals(float num1, float num2);
    static bool realNumberEquals(double num1, double num2);

    static bool matEquals(cv::Mat mat1, cv::Mat mat2);
};


#endif //ORB_SLAM2_SERIALIZATIONTESTER_H
