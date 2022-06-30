#include<Eigen/Dense>
#include "VoxelCarving.h"
#include "PoseEstimation.h"
#include "Segmentation.h"
#include <opencv2/core/eigen.hpp>

using cv::Point;
using cv::Vec3b;
using cv::Scalar;

static cv::Vec3f worldToCamera(int x, int y, int z, cv::Mat pose, cv::Mat intr) {
	cv::Mat1f proj = intr * pose * cv::Vec4f(x, y, z, 1);
	return cv::Vec3f(proj(0) / proj(2), proj(1) / proj(2), 1);
}

static Model carve(cv::Mat cameraMatrix, cv::Mat distCoeffs, Model model, cv::Mat image) {
	// Estimate camera extrinsics
	cv::Mat4f pose = estimatePoseFromImage(cameraMatrix, distCoeffs, image, false);

	// Generate carcing mask
	cv::Mat mask = color_segmentation(image);

	for (int x = 0; x < model.getX(); x++)
	{
		for (int y = 0; y < model.getY(); y++)
		{
			for (int z = 0; z < model.getZ(); z++)
			{
				// for each voxel check if corresponding pixel is solid or background
				cv::Vec3f camera_coord = worldToCamera(x, y, z, pose(cv::Rect(0, 0, 4, 3)), cameraMatrix);
				Vec3b pixel = mask.at<Vec3b>(Point((int) camera_coord[0], (int) camera_coord[1]));
				if (Scalar(0, 0, 0) == Scalar(pixel)) // masked pixel -> set alpha = 0
				{
					model.get(x, y, z)(4) = 0;
				}

			}
		}
	}

	return model;
};

static Model carve(cv::Mat cameraMatrix, cv::Mat distCoeffs, Model model, std::vector<cv::Mat> images) {
	for (cv::Mat image : images)
		carve(cameraMatrix, distCoeffs, model, image);
	return model;
};