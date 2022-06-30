#pragma once
#include "Model.h"
#include <opencv2/core/mat.hpp>

static void carve(cv::Mat* cameraMatrix, cv::Mat* distCoeffs, Model* model, cv::Mat* image);
static void carve(cv::Mat* cameraMatrix, cv::Mat* distCoeffs, Model* model, std::vector<cv::Mat>* images);