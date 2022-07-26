#pragma once
#include <opencv2/imgproc.hpp>

inline cv::Mat color_segmentation(cv::Mat Input){
    cv::Mat rgb_img, hsv_img;
    cv::cvtColor(Input, rgb_img, cv::COLOR_BGR2RGB);
    cv::cvtColor(Input, hsv_img, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    inRange(rgb_img, cv::Scalar(120, 120, 120), cv::Scalar(255, 255, 255), mask);
    return ~mask;
}

inline cv::Mat kmeans_segmentation(cv::Mat Input) {
cv::Mat samples(Input.rows * Input.cols, Input.channels(), CV_32F);
for (int y = 0; y < Input.rows; y++)
    for (int x = 0; x < Input.cols; x++)
        for (int z = 0; z < Input.channels(); z++)
            if (Input.channels() == 3) {
                samples.at<float>(y + x * Input.rows, z) = Input.at<cv::Vec3b>(y, x)[z];
            }
            else {
                samples.at<float>(y + x * Input.rows, z) = Input.at<uchar>(y, x);
            }

cv::Mat labels;
int attempts = 5;
cv::Mat centers;
int K=2;
cv::kmeans(samples, K, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10000, 0.0001), attempts, cv::KMEANS_PP_CENTERS, centers);


cv::Mat new_image(Input.size(), Input.type());
for (int y = 0; y < Input.rows; y++)
    for (int x = 0; x < Input.cols; x++)
    {
        int cluster_idx = labels.at<int>(y + x * Input.rows, 0);
        if (Input.channels()==3) {
            for (int i = 0; i < Input.channels(); i++) {
                new_image.at<cv::Vec3b>(y, x)[i] = centers.at<float>(cluster_idx, i);
            }
        }
        else {
            new_image.at<uchar>(y, x) = centers.at<float>(cluster_idx, 0);
        }
    }
return new_image;
}
