
#include "vision.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>

#include <chrono>
#include <iostream>
#include <vector>
#include <string>

// Generate ArUco marker
bool makeMarker(const std::string& path, int id) {

    if (id < 0 || id >= 250)
        return false;

    cv::Mat marker;

    auto dict = cv::aruco::getPredefinedDictionary(
        cv::aruco::DICT_6X6_250
    );

    cv::aruco::generateImageMarker(
        dict, id, 500, marker, 1
    );

    return cv::imwrite(path, marker);
}

// Detect marker and estimate its position
MarkerPose detectMarker(
    const cv::Mat& frame,
    const cv::Mat& k,
    const cv::Mat& distortion,
    double markerSize,
    cv::Mat& annotated
) {

    MarkerPose result;

    auto start = std::chrono::steady_clock::now();

    if (frame.empty() || k.empty() || markerSize <= 0)
        return result;

    annotated = frame.clone();

    auto dict = cv::aruco::getPredefinedDictionary(
        cv::aruco::DICT_6X6_250
    );

    cv::aruco::ArucoDetector detector(dict);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    detector.detectMarkers(frame, corners, ids);

    if (!ids.empty()) {

        cv::aruco::drawDetectedMarkers(
            annotated, corners, ids
        );

        float h = static_cast<float>(markerSize / 2.0);

        std::vector<cv::Point3f> objectPts = {
            {-h,  h, 0},
            { h,  h, 0},
            { h, -h, 0},
            {-h, -h, 0}
        };

        cv::Vec3d rvec, tvec;

        bool ok = cv::solvePnP(
            objectPts,
            corners[0],
            k,
            distortion,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_IPPE_SQUARE
        );

        if (ok) {
            result.detected = true;
            result.id = ids[0];
            result.rotation = rvec;
            result.translation = tvec;

            cv::drawFrameAxes(
                annotated,
                k,
                distortion,
                rvec,
                tvec,
                static_cast<float>(markerSize * 0.5)
            );
        }
    }

    result.processingMs =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start
        ).count();

    return result;
}

// Load saved camera calibration
bool loadCalibration(
    const std::string& path,
    cv::Mat& k,
    cv::Mat& distortion
) {

    cv::FileStorage fs(path, cv::FileStorage::READ);

    if (!fs.isOpened())
        return false;

    fs["camera_matrix"] >> k;
    fs["distortion_coefficients"] >> distortion;

    return !k.empty() && !distortion.empty();
}

// Camera calibration using chessboard images
bool calibrateCameraFromImages(
    const std::string& pattern,
    const std::string& output,
    int cols,
    int rows,
    double square
) {

    if (cols <= 0 || rows <= 0 || square <= 0)
        return false;

    std::vector<cv::String> files;
    cv::glob(pattern, files, false);

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    cv::Size imageSize;

    std::vector<cv::Point3f> board;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            board.emplace_back(
                static_cast<float>(x * square),
                static_cast<float>(y * square),
                0.0f
            );
        }
    }

    for (const auto& file : files) {

        cv::Mat img = cv::imread(file);

        if (img.empty())
            continue;

        if (!imageSize.empty() && img.size() != imageSize)
            continue;

        cv::Mat gray;

        cv::cvtColor(
            img,
            gray,
            cv::COLOR_BGR2GRAY
        );

        std::vector<cv::Point2f> corners;

        // OpenCV 5 compatible checkerboard detection
        bool found = cv::findChessboardCornersSB(
            gray,
            cv::Size(cols, rows),
            corners
        );

        if (found) {

            imageSize = img.size();

            imagePoints.push_back(corners);
            objectPoints.push_back(board);
        }
    }

    if (imagePoints.size() < 8) {

        std::cerr
            << "Need at least 8 valid checkerboard images; found "
            << imagePoints.size()
            << "\n";

        return false;
    }

    cv::Mat k, dist;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        k,
        dist,
        rvecs,
        tvecs
    );

    cv::FileStorage fs(output, cv::FileStorage::WRITE);

    if (!fs.isOpened())
        return false;

    fs << "camera_matrix" << k;
    fs << "distortion_coefficients" << dist;
    fs << "rms_reprojection_error" << rms;

    std::cout
        << "Calibration RMS: "
        << rms
        << " pixels from "
        << imagePoints.size()
        << " images\n";

    return true;
}
