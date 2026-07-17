#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/ml.hpp>

#include <string>
#include <vector>

namespace robot::vision
{
struct LostItemDetection
{
    cv::Rect bounding_box;
    float margin = 0.0F;
    float threshold = 0.0F;
};

struct LostItemResult
{
    std::vector<LostItemDetection> items;
    cv::Mat annotated_frame;
    std::string error;
    std::string toJson() const;
};

class LostItemDetector
{
public:
    bool load(const std::string& model_path, const std::string& metadata_path);
    LostItemResult detect(const cv::Mat& frame, int max_candidates = 5) const;
    LostItemDetection classifyCrop(const cv::Mat& crop) const;
    const std::string& error() const { return error_; }

private:
    cv::Mat letterbox(const cv::Mat& image) const;
    float thresholdFor(const cv::Mat& crop) const;

    cv::Ptr<cv::ml::SVM> svm_;
    float raw_sign_ = -1.0F;
    float decision_threshold_ = -0.30F;
    float near_min_aspect_ = 2.2F;
    float near_width_reference_ = 145.0F;
    float near_threshold_slope_ = 0.006F;
    float near_threshold_floor_ = -0.95F;
    std::string error_;
};
} // namespace robot::vision
