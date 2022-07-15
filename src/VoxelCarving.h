#pragma once

#ifndef VOXEL_CARVING_H
#define VOXEL_CARVING_H


#include "Model.h"
#include <opencv2/core/mat.hpp>

/**
 * @brief This function carves a multiple frames out of the given model.
 *
 * @param cameraMatrix	camera intrinsics
 * @param distCoeffs	distortion coefficients
 * @param model			voxel model
 * @param images		colored images to carve
 * @param masks			segmentation masks
 */
void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks);

/**
 * @brief This function carves a multiple frames out of the given model. Using a more error prone but significantly faster greedy approach compared to the standard method.
 * @see carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks)
 *
 * @param cameraMatrix	camera intrinsics
 * @param distCoeffs	distortion coefficients
 * @param model			voxel model
 * @param images		colored images to carve
 * @param masks			segmentation masks
 */
void fastCarve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks);

#endif