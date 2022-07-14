#pragma once

#include "ColorReconstruction.h"
#include "aruco_samples_utility.hpp"
#include "PoseEstimation.h"
#include "Segmentation.h"
#include "Benchmark.h"

static cv::Vec3f worldToCamera(cv::Vec4f world, cv::Mat& pose, cv::Mat& intr) {
    cv::Mat1f proj = intr * pose * world;
    return cv::Vec3f(proj(0) / proj(2), proj(1) / proj(2), 1);
}

void reconstructColor(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - CR: starting color reconstruction." << std::endl;
    Benchmark::GetInstance().LogColoring(true);
    cv::Mat intr = cameraMatrix.clone();
    intr.convertTo(intr, CV_32F);

    // Estimate pose for each image and remove distortion from images/masks
    std::vector<cv::Vec4f> cameras;
    std::vector<cv::Mat> poses;
    std::vector<cv::Mat> undist_imgs;
    std::vector<cv::Mat> undist_masks;
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat pose = estimatePoseFromImage(cameraMatrix, distCoeffs, images[i], false);
        poses.push_back(pose.inv()(cv::Rect(0, 0, 4, 3)));
        cameras.push_back(cv::Vec4f(poses[i].at<float>(0, 3), poses[i].at<float>(1, 3), poses[i].at<float>(2, 3), 1));
        cv::Mat undist_img;
        cv::undistort(images[i], undist_img, cameraMatrix, distCoeffs);
        undist_imgs.push_back(undist_img);
        cv::Mat undist_mask;
        cv::undistort(masks[i], undist_mask, cameraMatrix, distCoeffs);
        undist_masks.push_back(undist_mask);
    }

    cv::Rect image_borders = cv::Rect(0, 0, images[0].cols, images[0].rows);

    for (int x = 0; x < model.getX(); x++)
    {
        for (int y = 0; y < model.getY(); y++)
        {
            for (int z = 0; z < model.getZ(); z++)
            {
                if (model.get(x, y, z)(3) == 0 ||  model.isInner(x, y, z))
                {
                    continue;
                }
                cv::Vec4f word_coord = model.toWord(x, y, z);
                for (int i = 0; i < undist_imgs.size(); i++) {
                    cv::Vec3f camera_coord = worldToCamera(word_coord, poses[i], intr);
                    cv::Point pixel_pos = cv::Point((int)std::round(camera_coord[0]), (int)std::round(camera_coord[1]));
                    if (!pixel_pos.inside(image_borders))
                    {
                        continue;
                    }
                    cv::Vec3b pixel = undist_imgs[i].at<cv::Vec3b>(pixel_pos);
                    // convert BGR pixel to RGB and add to color candidates
                    model.addColor(x, y, z, Eigen::Vector4f(pixel(2), pixel(1), pixel(0), 1), cv::norm(cameras[i] - word_coord));
                }
                std::vector<DCLR> colors = model.getColors(x, y, z);
                if (colors.size() == 0) {
                    continue;
                }
                DCLR min = colors[0];
                for (int i = 1; i < colors.size(); i++) {
                    if (colors[i].depth < min.depth)
                    {
                        min = colors[i];
                    }
                }
                model.set(x, y, z, min.color);
            }
        }
    }
    Benchmark::GetInstance().LogColoring(false);
    std::cout << "LOG - CR: color reconstruction finished." << std::endl;
}

void reconstructClosestColor(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - CR: starting color reconstruction (closest color)." << std::endl;
    Benchmark::GetInstance().LogColoring(true);
    int x = 0, y = 0, z = 0;
    voxel_pass(x, y, z, cameraMatrix, distCoeffs, model, images, masks)
    {   
        std::vector<DCLR> colors = model.getColors(x, y, z);
        if (colors.size() == 0) {
            continue;
        }
        DCLR min = colors[0];
        for (int i = 1; i < colors.size(); i++) {
            if (colors[i].depth < min.depth)
            {
                min = colors[i];
            }
        }
        model.set(x, y, z, min.color);

    }
    Benchmark::GetInstance().LogColoring(false);
    std::cout << "LOG - CR: color reconstruction finished." << std::endl;
}

void reconstructAvgColor(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - CR: starting color reconstruction (average color)." << std::endl;
    Benchmark::GetInstance().LogColoring(true);
    int x = 0, y = 0, z = 0;
    voxel_pass(x, y, z, cameraMatrix, distCoeffs, model, images, masks)
    {
        std::vector<DCLR> colors = model.getColors(x, y, z);
        if (colors.size() == 0) {
            continue;
        }
        Vector4f sum = Vector4f(0, 0, 0, 1);
        for (DCLR& c : colors)
        {
            sum = sum + c.color;
        }
        Vector4f avg = sum / colors.size();
        model.set(x, y, z, Vector4f(std::round(avg.x()), std::round(avg.y()), std::round(avg.z()), 1));
    }
    Benchmark::GetInstance().LogColoring(false);
    std::cout << "LOG - CR: color reconstruction finished." << std::endl;
}