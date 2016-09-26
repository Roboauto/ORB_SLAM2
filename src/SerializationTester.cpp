/**
 * Created by Matous Kolarik on 9/23/16.
 */

#include <iostream>
#include <opencv-3.1.0-dev/opencv2/core.hpp>
#include "SerializationTester.h"

bool SerializationTester::realNumberEquals(float num1, float num2) {
    return fabs(num1 - num2) < MAX_REAL_NUM_DIFF;
}

bool SerializationTester::realNumberEquals(double num1, double num2) {
    return fabs(num1 - num2) < MAX_REAL_NUM_DIFF;
}

bool SerializationTester::matEquals(cv::Mat mat1, cv::Mat mat2) {
    if (mat1.rows != mat2.rows || mat1.cols != mat2.cols) {
        return false;
    } else {
        cv::Mat diff = mat1 - mat2;
        if (cv::countNonZero(diff) != 0) { //matrices are not identical
            return false;
        }
    }
    return true;
}
