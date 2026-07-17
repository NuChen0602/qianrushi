#include "vision/detection_tracker.h"

#include <algorithm>

namespace robot::vision
{
DetectionTracker::DetectionTracker(int confirmation, int missed, double overlap)
    : confirmation_frames_(std::max(1, confirmation)), maximum_missed_frames_(std::max(1, missed)),
      minimum_iou_(std::max(0.0, overlap)) {}
double DetectionTracker::iou(const cv::Rect& a, const cv::Rect& b)
{
    const double intersection = (a & b).area();
    const double total = a.area() + b.area() - intersection;
    return total > 0.0 ? intersection / total : 0.0;
}
const std::vector<TrackedDetection>& DetectionTracker::update(const std::vector<cv::Rect>& boxes)
{
    std::vector<bool> used(boxes.size(), false);
    for(auto& track : tracks_)
    {
        int best = -1; double overlap = minimum_iou_;
        for(std::size_t i = 0; i < boxes.size(); ++i)
            if(!used[i] && iou(track.box, boxes[i]) >= overlap) { overlap = iou(track.box, boxes[i]); best = static_cast<int>(i); }
        if(best >= 0)
        {
            track.box = boxes[best]; track.missed_frames = 0; ++track.positive_frames;
            track.confirmed = track.positive_frames >= confirmation_frames_; used[best] = true;
        }
        else ++track.missed_frames;
    }
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const auto& track)
        { return track.missed_frames > maximum_missed_frames_; }), tracks_.end());
    for(std::size_t i = 0; i < boxes.size(); ++i)
        if(!used[i]) tracks_.push_back({next_id_++, boxes[i], confirmation_frames_ <= 1, 1, 0});
    return tracks_;
}
} // namespace robot::vision
