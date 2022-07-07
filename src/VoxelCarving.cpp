#pragma once
#include "aruco_samples_utility.hpp"
#include "PoseEstimation.h"
#include "VoxelCarving.h"
#include "Segmentation.h"

static cv::Vec4f modelToWorld(int x, int y, int z) {
    return cv::Vec4f(x * 0.0028, y * 0.0028, z * -0.0028, 1);
    //return cv::Vec4f(x * 0.28, y * 0.28, z * 0.28, 1);
}

static cv::Vec3f worldToCamera(int x, int y, int z, cv::Mat& pose, cv::Mat& intr) {
    // cv::Mat1f proj = intr * pose * cv::Vec4f(x, y, z, 1);
    cv::Mat1f proj = intr * pose * modelToWorld(x, y, z);
    return cv::Vec3f(proj(0) / proj(2), proj(1) / proj(2), 1);
}

static void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, cv::Mat& image, cv::Mat& mask) {
    int carved = 0;
    // Estimate camera extrinsics
    // cv::Mat4f
    cv::Mat pose = estimatePoseFromImage(cameraMatrix, distCoeffs, image, false);
    std::cout << "LOG - VC: finished pose estimation." << std::endl;
    pose = pose.inv();

    cv::Mat intr = cameraMatrix.clone();
    intr.convertTo(intr, CV_32F);
    std::cout << "LOG - VC: generated intrinsics." << std::endl;

    cv::Rect image_borders = cv::Rect(0, 0, image.cols, image.rows);
    cv::Mat undist_img;
    cv::undistort(image, undist_img, cameraMatrix, distCoeffs);
    cv::Mat undist_mask;
    cv::undistort(mask, undist_mask, cameraMatrix, distCoeffs);

    std::cout << "LOG - VC: Testing Proj start." << std::endl;

    cv::Mat image_cpy = undist_img.clone();
    cv::Mat subm = pose(cv::Rect(0, 0, 4, 3));
    cv::Vec3f camera_coordd = worldToCamera(0, 0, 0, subm, intr);
    cv::Point pixel_poss = cv::Point((int)std::round(camera_coordd[0]), (int)std::round(camera_coordd[1]));
    if (pixel_poss.inside(image_borders))
    {
        image_cpy.at<cv::Vec3b>(pixel_poss) = cv::Vec3b(20, 20, 180);
        cv::Mat diff;
        cv::absdiff(undist_img, image_cpy, diff);
        cv::imwrite("./out/debug_projection.jpg", diff);
    }

    std::cout << "LOG - VC: Testing Proj end." << std::endl;

    

    for (int x = 0; x < model.getX(); x++)
    {
        for (int y = 0; y < model.getY(); y++)
        {
            for (int z = 0; z < model.getZ(); z++)
            {
                // for each voxel check if corresponding pixel is solid or background
                /*
                pose = cv::Mat::eye(4, 4, CV_32F);
                pose.at<float>(0, 3) = -0.1;
                pose.at<float>(1, 3) = -0.1;
                pose.at<float>(2, 3) = -0.1;
                pose.at<float>(0, 0) = 0;
                pose.at<float>(1, 1) = 0;
                pose.at<float>(0, 1) = 1;
                pose.at<float>(1, 0) = -1;
                */
                cv::Mat subm = pose(cv::Rect(0, 0, 4, 3));
                cv::Vec3f camera_coord = worldToCamera(x, y, z, subm, intr);
                cv::Point pixel_pos = cv::Point((int)std::round(camera_coord[0]), (int)std::round(camera_coord[1]));
                if (!pixel_pos.inside(image_borders))
                {
                    continue;
                }
                cv::Vec3b pixel = undist_mask.at<cv::Vec3b>(pixel_pos);
                if (pixel(0) == 0 && pixel(1) == 0 && pixel(2) == 0) // masked pixel -> set alpha = 0
                {
                    carved++;
                    model.set(x, y, z, Eigen::Vector4f(0, 0, 0, 0));
                }
                model.visit(x, y, z);
            }
        }
    }

    std::cout << "LOG - VC: completed carving of a single image." << carved << std::endl;
}

void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - VC: starting carving process." << std::endl;
    for (int i = 0; i < images.size(); i++)
        carve(cameraMatrix, distCoeffs, model, images[i], masks[i]);
    std::cout << "LOG - VC: carving complete." << std::endl;
}