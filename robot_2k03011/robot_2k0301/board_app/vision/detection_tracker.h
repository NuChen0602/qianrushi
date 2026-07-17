#pragma once

#include <opencv2/core/types.hpp>

#include <vector>

namespace robot::vision
{
struct TrackedDetection
{
    int track_id = 0;
    cv::Rect box;
    bool confirmed = false;
    int positive_frames = 0, missed_frames = 0;
};

class DetectionTracker
{
public:
    DetectionTracker(int confirmation_frames = 3, int maximum_missed_frames = 4,
                     double minimum_iou = 0.12);
    const std::vector<TrackedDetection>& update(const std::vector<cv::Rect>& positive_boxes);
    const std::vector<TrackedDetection>& tracks() const { return tracks_; }
private:
    static double iou(const cv::Rect& first, const cv::Rect& second);
    int confirmation_frames_, maximum_missed_frames_, next_id_ = 1;
    double minimum_iou_;
    std::vector<TrackedDetection> tracks_;
};
} // namespace robot::vision
