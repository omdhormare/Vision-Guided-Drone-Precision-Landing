#pragma once
#include <opencv2/core.hpp>
#include <string>
struct MarkerPose { bool detected=false; int id=-1; cv::Vec3d translation{0,0,0}; cv::Vec3d rotation{0,0,0}; double processingMs=0; };
bool makeMarker(const std::string& path, int markerId=23);
MarkerPose detectMarker(const cv::Mat& frame, const cv::Mat& cameraMatrix, const cv::Mat& distortion, double markerSizeMeters, cv::Mat& annotated);
bool calibrateCameraFromImages(const std::string& globPattern, const std::string& outputFile, int cols, int rows, double squareMeters);
bool loadCalibration(const std::string& path, cv::Mat& matrix, cv::Mat& distortion);
