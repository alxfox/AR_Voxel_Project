#pragma once
#include "aruco_samples_utility.hpp"
#include "PoseEstimation.h"
#include "VoxelCarving.h"
#include "Segmentation.h"

static cv::Vec3f worldToCamera(int x, int y, int z, cv::Mat* pose, cv::Mat* intr) {
    cv::Mat1f proj = *intr * *pose * cv::Vec4f(x, y, z, 1);
    return cv::Vec3f(proj(0) / proj(2), proj(1) / proj(2), 1);
}

static void carve(cv::Mat* cameraMatrix, cv::Mat* distCoeffs, Model* model, cv::Mat* image) {
    // Estimate camera extrinsics
    cv::Mat4f pose = estimatePoseFromImage(*cameraMatrix, *distCoeffs, *image, false);


    // Generate carcing mask
    cv::Mat mask = color_segmentation(*image);

    for (int x = 0; x < model->getX(); x++)
    {
        for (int y = 0; y < model->getY(); y++)
        {
            for (int z = 0; z < model->getZ(); z++)
            {
                // for each voxel check if corresponding pixel is solid or background
                cv::Mat subm = pose(cv::Rect(0, 0, 4, 3));
                cv::Vec3f camera_coord = worldToCamera(x, y, z, &subm, cameraMatrix);
                cv::Vec3b pixel = mask.at<cv::Vec3b>(cv::Point((int) camera_coord[0], (int) camera_coord[1]));
                if (cv::Scalar(0, 0, 0) == cv::Scalar(pixel)) // masked pixel -> set alpha = 0
                {
                    model->get(x, y, z)(4) = 0;
                }

            }
        }
    }

}

static void carve(cv::Mat* cameraMatrix, cv::Mat* distCoeffs, Model* model, std::vector<cv::Mat>* images) {
    for (cv::Mat image : *images)
        carve(cameraMatrix, distCoeffs, model, &image);
}