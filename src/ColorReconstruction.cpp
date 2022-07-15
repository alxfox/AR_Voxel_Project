#pragma once

#include "ColorReconstruction.h"
#include "aruco_samples_utility.hpp"
#include "PoseEstimation.h"
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

        // find color with min depth
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

        // calculate average color value
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
