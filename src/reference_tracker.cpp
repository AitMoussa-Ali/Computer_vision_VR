#include "reference_tracker.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace q1 {
namespace {

constexpr int kMaxFeatures = 1800;
constexpr int kRedetectionInterval = 10;
constexpr int kMinimumInliers = 8;
constexpr float kRatioThreshold = 0.75F;
constexpr double kRansacThreshold = 3.0;

bool validCorners(const std::vector<cv::Point2f>& corners, const cv::Size& frame_size)
{
    if (corners.size() != 4 || frame_size.area() <= 0) {
        return false;
    }

    for (const cv::Point2f& point : corners) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            return false;
        }
    }

    if (!cv::isContourConvex(corners)) {
        return false;
    }

    const double area = std::abs(cv::contourArea(corners));
    const double frame_area = static_cast<double>(frame_size.area());
    return area >= frame_area * 0.0005 && area <= frame_area * 0.95;
}

} // namespace

ReferenceTracker::ReferenceTracker(const std::vector<std::string>& image_paths)
    : orb_(cv::ORB::create(kMaxFeatures))
{
    if (image_paths.empty()) {
        throw std::runtime_error("Aucune image de reference n'a ete fournie.");
    }

    references_.reserve(image_paths.size());
    tracks_.resize(image_paths.size());

    for (const std::string& path : image_paths) {
        cv::Mat gray = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (gray.empty()) {
            throw std::runtime_error("Impossible de lire l'image de reference : " + path);
        }

        ReferenceImage reference;
        const std::size_t separator = path.find_last_of("/\\");
        const std::size_t filename_start =
            separator == std::string::npos ? 0 : separator + 1;
        const std::size_t extension = path.find_last_of('.');
        const std::size_t filename_end =
            extension == std::string::npos || extension < filename_start
                ? path.size()
                : extension;
        reference.name = path.substr(filename_start, filename_end - filename_start);
        reference.size = gray.size();
        orb_->detectAndCompute(gray, cv::noArray(), reference.keypoints, reference.descriptors);
        if (reference.descriptors.empty() || reference.keypoints.size() < kMinimumInliers) {
            throw std::runtime_error("Pas assez de points d'interet dans l'image : " + path);
        }

        references_.push_back(std::move(reference));
    }
}

std::vector<Detection> ReferenceTracker::update(const cv::Mat& frame)
{
    if (frame.empty()) {
        throw std::runtime_error("L'image video courante est vide.");
    }

    cv::Mat current_gray;
    if (frame.channels() == 1) {
        current_gray = frame;
    } else {
        cv::cvtColor(frame, current_gray, cv::COLOR_BGR2GRAY);
    }

    for (std::size_t i = 0; i < references_.size(); ++i) {
        if (tracks_[i].active && !previous_gray_.empty() &&
            !track(references_[i], current_gray, tracks_[i])) {
            tracks_[i].active = false;
        }
    }

    if (frame_index_ % kRedetectionInterval == 0) {
        std::vector<cv::KeyPoint> frame_keypoints;
        cv::Mat frame_descriptors;
        orb_->detectAndCompute(current_gray, cv::noArray(), frame_keypoints, frame_descriptors);

        if (!frame_descriptors.empty()) {
            for (std::size_t i = 0; i < references_.size(); ++i) {
                TrackState detected_state;
                if (detect(references_[i], frame_keypoints, frame_descriptors,
                           current_gray.size(), detected_state)) {
                    tracks_[i] = std::move(detected_state);
                }
            }
        }
    }

    std::vector<Detection> detections;
    for (std::size_t i = 0; i < references_.size(); ++i) {
        if (tracks_[i].active) {
            detections.push_back({references_[i].name, tracks_[i].corners});
        }
    }

    previous_gray_ = current_gray.clone();
    ++frame_index_;
    return detections;
}

bool ReferenceTracker::detect(const ReferenceImage& reference,
                              const std::vector<cv::KeyPoint>& frame_keypoints,
                              const cv::Mat& frame_descriptors,
                              const cv::Size& frame_size,
                              TrackState& state) const
{
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> nearest_matches;
    matcher.knnMatch(reference.descriptors, frame_descriptors, nearest_matches, 2);

    std::vector<cv::Point2f> reference_points;
    std::vector<cv::Point2f> frame_points;
    for (const std::vector<cv::DMatch>& candidates : nearest_matches) {
        if (candidates.size() == 2 &&
            candidates[0].distance < kRatioThreshold * candidates[1].distance) {
            reference_points.push_back(reference.keypoints[candidates[0].queryIdx].pt);
            frame_points.push_back(frame_keypoints[candidates[0].trainIdx].pt);
        }
    }

    if (reference_points.size() < kMinimumInliers) {
        return false;
    }

    cv::Mat inlier_mask;
    const cv::Mat homography = cv::findHomography(
        reference_points, frame_points, cv::RANSAC, kRansacThreshold, inlier_mask);
    if (homography.empty()) {
        return false;
    }

    std::vector<unsigned char> inliers(inlier_mask.total());
    for (std::size_t i = 0; i < inliers.size(); ++i) {
        inliers[i] = inlier_mask.at<unsigned char>(static_cast<int>(i));
    }

    const int inlier_count = cv::countNonZero(inlier_mask);
    if (inlier_count < kMinimumInliers ||
        static_cast<double>(inlier_count) / reference_points.size() < 0.3) {
        return false;
    }

    return estimateTrack(reference, reference_points, frame_points, inliers,
                         frame_size, state);
}

bool ReferenceTracker::track(const ReferenceImage& reference,
                             const cv::Mat& current_gray,
                             TrackState& state) const
{
    std::vector<cv::Point2f> next_points;
    std::vector<unsigned char> status;
    std::vector<float> errors;
    cv::calcOpticalFlowPyrLK(previous_gray_, current_gray, state.frame_points,
                             next_points, status, errors);

    std::vector<cv::Point2f> reference_points;
    std::vector<cv::Point2f> frame_points;
    for (std::size_t i = 0; i < status.size(); ++i) {
        if (status[i] && errors[i] <= 20.0F) {
            reference_points.push_back(state.reference_points[i]);
            frame_points.push_back(next_points[i]);
        }
    }

    if (reference_points.size() < kMinimumInliers) {
        return false;
    }

    cv::Mat inlier_mask;
    const cv::Mat homography = cv::findHomography(
        reference_points, frame_points, cv::RANSAC, kRansacThreshold, inlier_mask);
    if (homography.empty() || cv::countNonZero(inlier_mask) < kMinimumInliers) {
        return false;
    }

    std::vector<unsigned char> inliers(inlier_mask.total());
    for (std::size_t i = 0; i < inliers.size(); ++i) {
        inliers[i] = inlier_mask.at<unsigned char>(static_cast<int>(i));
    }

    return estimateTrack(reference, reference_points, frame_points, inliers,
                         current_gray.size(), state);
}

bool ReferenceTracker::estimateTrack(
    const ReferenceImage& reference,
    const std::vector<cv::Point2f>& reference_points,
    const std::vector<cv::Point2f>& frame_points,
    const std::vector<unsigned char>& inlier_mask,
    const cv::Size& frame_size,
    TrackState& state) const
{
    state.reference_points.clear();
    state.frame_points.clear();
    for (std::size_t i = 0; i < inlier_mask.size(); ++i) {
        if (inlier_mask[i]) {
            state.reference_points.push_back(reference_points[i]);
            state.frame_points.push_back(frame_points[i]);
        }
    }

    if (state.reference_points.size() < kMinimumInliers) {
        return false;
    }

    const std::vector<cv::Point2f> reference_corners = {
        {0.0F, 0.0F},
        {static_cast<float>(reference.size.width - 1), 0.0F},
        {static_cast<float>(reference.size.width - 1),
         static_cast<float>(reference.size.height - 1)},
        {0.0F, static_cast<float>(reference.size.height - 1)}
    };

    const cv::Mat homography = cv::findHomography(
        state.reference_points, state.frame_points, cv::RANSAC, kRansacThreshold);
    if (homography.empty()) {
        return false;
    }

    cv::perspectiveTransform(reference_corners, state.corners, homography);
    if (!validCorners(state.corners, frame_size)) {
        state.corners.clear();
        return false;
    }

    state.active = true;
    return true;
}

} // namespace q1
