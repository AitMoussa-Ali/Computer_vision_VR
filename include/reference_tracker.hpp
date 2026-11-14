#pragma once

#include <opencv2/features2d.hpp>

#include <string>
#include <vector>

namespace q1 {

struct Detection {
    std::string name;
    std::vector<cv::Point2f> corners;
};

class ReferenceTracker {
public:
    explicit ReferenceTracker(const std::vector<std::string>& image_paths);

    std::vector<Detection> update(const cv::Mat& frame);

private:
    struct ReferenceImage {
        std::string name;
        cv::Size size;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
    };

    struct TrackState {
        bool active = false;
        std::vector<cv::Point2f> reference_points;
        std::vector<cv::Point2f> frame_points;
        std::vector<cv::Point2f> corners;
    };

    bool detect(const ReferenceImage& reference,
                const std::vector<cv::KeyPoint>& frame_keypoints,
                const cv::Mat& frame_descriptors,
                const cv::Size& frame_size,
                TrackState& state) const;
    bool track(const ReferenceImage& reference,
               const cv::Mat& current_gray,
               TrackState& state) const;
    bool estimateTrack(const ReferenceImage& reference,
                       const std::vector<cv::Point2f>& reference_points,
                       const std::vector<cv::Point2f>& frame_points,
                       const std::vector<unsigned char>& inlier_mask,
                       const cv::Size& frame_size,
                       TrackState& state) const;

    std::vector<ReferenceImage> references_;
    std::vector<TrackState> tracks_;
    cv::Ptr<cv::ORB> orb_;
    cv::Mat previous_gray_;
    std::size_t frame_index_ = 0;
};

} // namespace q1
