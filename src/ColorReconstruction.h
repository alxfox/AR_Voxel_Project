#pragma once

#ifndef COLOR_RECONSTRUCTION_H
#define COLOR_RECONSTRUCTION_H

#include "Utils.h"
#include "Model.h"
#include <opencv2/core/mat.hpp>

#define voxel_pass(x, y, z, cameraMatrix, distCoeffs, model, images, masks) \
    cv::Mat intr = cameraMatrix.clone(); \
    intr.convertTo(intr, CV_32F); \
    std::vector<cv::Vec4f> cameras; \
    std::vector<cv::Mat> poses; \
    std::vector<cv::Mat> undist_imgs; \
    std::vector<cv::Mat> undist_masks; \
    for (int i = 0; i < images.size(); i++) \
    { \
        cv::Mat pose = estimatePoseFromImage(cameraMatrix, distCoeffs, images[i], false); \
        poses.push_back(pose.inv()(cv::Rect(0, 0, 4, 3))); \
        cameras.push_back(cv::Vec4f(poses[i].at<float>(0, 3), poses[i].at<float>(1, 3), poses[i].at<float>(2, 3), 1)); \
        cv::Mat undist_img; \
        cv::undistort(images[i], undist_img, cameraMatrix, distCoeffs); \
        undist_imgs.push_back(undist_img); \
        cv::Mat undist_mask; \
        cv::undistort(masks[i], undist_mask, cameraMatrix, distCoeffs); \
        undist_masks.push_back(undist_mask); \
    } \
    cv::Rect image_borders = cv::Rect(0, 0, images[0].cols, images[0].rows); \
    voxel_pass_loops(x, y, z, cameraMatrix, distCoeffs, model, images, masks)

#define voxel_pass_loops(x, y, z, cameraMatrix, distCoeffs, model, images, masks) \
    if (1) \
    { \
        x = 0; \
        for (; x < model.getX(); x++) \
        { \
            y = 0; \
            for (; y < model.getY(); y++) \
            { \
                z = 0; \
                for (; z < model.getZ(); z++) \
                { \
                    if (model.get(x, y, z)(3) == 0 || model.isInner(x, y, z)) \
                    { \
                        continue; \
                    } \
                    cv::Vec4f word_coord = model.toWord(x, y, z); \
                    for (int i = 0; i < undist_imgs.size(); i++) { \
                        cv::Vec3f camera_coord = worldToCamera(word_coord, poses[i], intr); \
                        cv::Point pixel_pos = cv::Point((int)std::round(camera_coord[0]), (int)std::round(camera_coord[1])); \
                        if (!pixel_pos.inside(image_borders)) \
                        { \
                            continue; \
                        } \
                        cv::Vec3b pixel = undist_imgs[i].at<cv::Vec3b>(pixel_pos); \
                        model.addColor(x, y, z, Eigen::Vector4f(pixel(2), pixel(1), pixel(0), 1), cv::norm(cameras[i] - word_coord)); \
                    } \
                    goto label(body, __LINE__); \
                    label(loop_continue, __LINE__): ; \
                } \
            } \
        } \
    } \
    else \
        while(1) \
            if(1) \
            { \
                goto label(loop_continue, __LINE__); \
            } \
            else \
                label(body, __LINE__):

/**
 * @brief This function performs color reconstruction choosing the closest observer.
 *
 * @param cameraMatrix	camera intrinsics
 * @param distCoeffs	distortion coefficients
 * @param model			voxel model
 * @param images		colored images to carve
 * @param masks			segmentation masks
 */
void reconstructClosestColor(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks);

/**
 * @brief This function performs color reconstruction averaging every observed color.
 *
 * @param cameraMatrix	camera intrinsics
 * @param distCoeffs	distortion coefficients
 * @param model			voxel model
 * @param images		colored images to carve
 * @param masks			segmentation masks
 */
void reconstructAvgColor(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks);

#endif