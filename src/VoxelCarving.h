#pragma once

#ifndef VOXEL_CARVING_H
#define VOXEL_CARVING_H


#include "Model.h"
#include <opencv2/core/mat.hpp>

static void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, cv::Mat& image);
void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images);

#endif