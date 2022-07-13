#pragma once

#ifndef COLOR_RECONSTRUCTION_H
#define COLOR_RECONSTRUCTION_H


#include "Model.h"
#include <opencv2/core/mat.hpp>

void reconstructColor(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks);
#endif