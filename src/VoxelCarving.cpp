#pragma once
#include <queue>
#include "aruco_samples_utility.hpp"
#include "PoseEstimation.h"
#include "VoxelCarving.h"
#include "Segmentation.h"
#include "Benchmark.h"

/**
 * @brief This function performs a projection from world to camera coordinates.
 *
 * @param world         homogenous world coordinates
 * @param pose          pose transformation
 * @param intr          camera intrinsics
 * @return cv::Vec3f    homogenous camera coordinates
 */
static cv::Vec3f worldToCamera(cv::Vec4f world, cv::Mat& pose, cv::Mat& intr) {
    cv::Mat1f proj = intr * pose * world;
    return cv::Vec3f(proj(0) / proj(2), proj(1) / proj(2), 1);
}

static void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, cv::Mat& image, cv::Mat& mask) {
    // Estimate camera extrinsics
    cv::Mat pose = estimatePoseFromImage(cameraMatrix, distCoeffs, image, false);
    pose = pose.inv();

    // Format camera intrinsics
    cv::Mat intr = cameraMatrix.clone();
    intr.convertTo(intr, CV_32F);

    cv::Rect image_borders = cv::Rect(0, 0, image.cols, image.rows);

    // remove distorions
    cv::Mat undist_mask;
    cv::undistort(mask, undist_mask, cameraMatrix, distCoeffs);

    int x, y, z;
    for_each_voxel(x, y, z) {
        // check if corresponding pixel is part of the object or background
        cv::Mat subm = pose(cv::Rect(0, 0, 4, 3));
        cv::Vec4f word_coord = model.toWord(x, y, z);
        cv::Vec3f camera_coord = worldToCamera(word_coord, subm, intr);
        cv::Point pixel_pos = cv::Point((int)std::round(camera_coord[0]), (int)std::round(camera_coord[1]));
        if (!pixel_pos.inside(image_borders))
        {
            continue;
        }
        cv::Vec3b pixel = undist_mask.at<cv::Vec3b>(pixel_pos);
        if (pixel(0) == 0 && pixel(1) == 0 && pixel(2) == 0) // masked pixel -> set alpha = 0
        {
            model.set(x, y, z, Eigen::Vector4f(0, 0, 0, 0));
        }
        model.see(x, y, z);
    }

    std::cout << "LOG - VC: completed carving of a single image." << std::endl;
}

void carve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - VC: starting carving process (version 1)." << std::endl;
    Benchmark::GetInstance().LogCarving(true);
    for (int i = 0; i < images.size(); i++) // carve each frame seperately
        carve(cameraMatrix, distCoeffs, model, images[i], masks[i]);
    Benchmark::GetInstance().LogCarving(false);
    std::cout << "LOG - VC: carving complete." << std::endl;
}

void fastCarve(cv::Mat& cameraMatrix, cv::Mat& distCoeffs, Model& model, std::vector<cv::Mat>& images, std::vector<cv::Mat>& masks) {
    std::cout << "LOG - VC: starting carving process (version 2)." << std::endl;
    Benchmark::GetInstance().LogCarving(true);
    // Estimate pose for each image and remove distortion from images/masks
    std::vector<cv::Mat> poses;
    std::vector<cv::Mat> undist_imgs;
    std::vector<cv::Mat> undist_masks;
    for (int i = 0; i < images.size(); i++)
    {
        poses.push_back(estimatePoseFromImage(cameraMatrix, distCoeffs, images[i], false).inv()(cv::Rect(0, 0, 4, 3)));

        cv::Mat undist_img;
        cv::undistort(images[i], undist_img, cameraMatrix, distCoeffs);
        undist_imgs.push_back(undist_img);
        cv::Mat undist_mask;
        cv::undistort(masks[i], undist_mask, cameraMatrix, distCoeffs);
        undist_masks.push_back(undist_mask);
    }

    // Format camera intrinsics
    cv::Mat intr = cameraMatrix.clone();
    intr.convertTo(intr, CV_32F);

    cv::Rect image_borders = cv::Rect(0, 0, images[0].cols, images[0].rows);

    // Greedy carve voxels
    std::queue<cv::Vec3i> q;
    q.push(cv::Vec3i(0, 0, 0));

    while (!q.empty())
    {
        cv::Vec3i current = q.front();
        q.pop();
        if (model.visited(current)) {
            continue;
        }
        model.visit(current);

        bool carved = false;
        for (int i = 0; i < images.size(); i++) // for each image check if voxel can be carved
        {
            // check if corresponding pixel is part of the object or background
            cv::Vec4f word_coord = model.toWord(current);
            cv::Vec3f camera_coord = worldToCamera(word_coord, poses[i], intr);
            cv::Point pixel_pos = cv::Point((int)std::round(camera_coord[0]), (int)std::round(camera_coord[1]));
            if (!pixel_pos.inside(image_borders))
            {
                continue;
            }
            cv::Vec3b pixel = undist_masks[i].at<cv::Vec3b>(pixel_pos);
            if (pixel(0) == 0 && pixel(1) == 0 && pixel(2) == 0) // masked pixel -> set alpha = 0
            {
                model.set(current, Eigen::Vector4f(0, 0, 0, 0));
                carved = true;
                break;
            }
        }

        if (carved) // if voxel was carved add newly revealed voxels to the queue
        {
            // x neighbours
            if (current(0) > 0 && !model.visited(cv::Vec3i(current(0) - 1, current(1), current(2)))) // x > 0
            {
                q.push(cv::Vec3i(current(0) - 1, current(1), current(2)));
            }
            if (current(0) < model.getX() - 1 && !model.visited(cv::Vec3i(current(0) + 1, current(1), current(2)))) // x < X - 1
            {
                q.push(cv::Vec3i(current(0) + 1, current(1), current(2)));
            }

            // y neighbours
            if (current(1) > 0 && !model.visited(cv::Vec3i(current(0), current(1) - 1, current(2)))) // y > 0
            {
                q.push(cv::Vec3i(current(0), current(1) - 1, current(2)));
            }
            if (current(1) < model.getY() - 1 && !model.visited(cv::Vec3i(current(0), current(1) + 1, current(2)))) // y < Y - 1
            {
                q.push(cv::Vec3i(current(0), current(1) + 1, current(2)));
            }

            // y neighbours
            if (current(2) > 0 && !model.visited(cv::Vec3i(current(0), current(1), current(2) - 1))) // z > 0
            {
                q.push(cv::Vec3i(current(0), current(1), current(2) - 1));
            }
            if (current(2) < model.getZ() - 1 && !model.visited(cv::Vec3i(current(0), current(1), current(2) + 1))) // z < Z - 1
            {
                q.push(cv::Vec3i(current(0), current(1), current(2) + 1));
            }
        }
    }
    Benchmark::GetInstance().LogCarving(false);
    std::cout << "LOG - VC: carving complete." << std::endl;
}