#include "vision/lost_item_detector.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace robot::vision
{
namespace
{
float readNumber(const std::string& json, const std::string& key, float fallback)
{
    const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    std::smatch match;
    return std::regex_search(json, match, pattern) ? std::stof(match[1].str()) : fallback;
}

double iou(const cv::Rect& a, const cv::Rect& b)
{
    const double intersection = (a & b).area();
    const double union_area = a.area() + b.area() - intersection;
    return union_area > 0.0 ? intersection / union_area : 0.0;
}

std::vector<cv::Rect> candidates(const cv::Mat& image, int limit)
{
    const int y0 = static_cast<int>(image.rows * 0.30);
    const cv::Mat roi = image(cv::Rect(0, y0, image.cols, image.rows - y0));
    cv::Mat gray, hsv, blurred, edges, saturation, background, difference, difference_mask, mask;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(gray, blurred, {5, 5}, 0);
    cv::Canny(blurred, edges, 35, 110);
    cv::inRange(hsv, cv::Scalar(0, 32, 0), cv::Scalar(180, 255, 255), saturation);
    cv::GaussianBlur(gray, background, {35, 35}, 0);
    cv::absdiff(gray, background, difference);
    cv::threshold(difference, difference_mask, 16, 255, cv::THRESH_BINARY);
    cv::bitwise_or(edges, saturation, mask);
    cv::bitwise_or(mask, difference_mask, mask);
    const auto close_kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel, {-1, -1}, 2);
    cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> boxes;
    const int image_area = image.cols * image.rows;
    for(const auto& contour : contours)
    {
        auto box = cv::boundingRect(contour);
        const int area = box.area();
        if(area < 300 || area > image_area * 0.55 || box.width < 16 || box.height < 12 ||
           box.width > image.cols * 0.95 || box.height > image.rows * 0.75) continue;
        const int padding = std::max(18, static_cast<int>(std::lround(std::max(box.width, box.height) * 0.08)));
        box.y += y0;
        const int x1 = std::max(0, box.x - padding);
        const int y1 = std::max(0, box.y - padding);
        const int x2 = std::min(image.cols, box.x + box.width + padding);
        const int y2 = std::min(image.rows, box.y + box.height + padding);
        cv::Rect expanded(x1, y1, x2 - x1, y2 - y1);
        bool merged = false;
        for(auto& current : boxes)
        {
            if(iou(expanded, current) > 0.12)
            {
                current |= expanded;
                merged = true;
                break;
            }
        }
        if(!merged) boxes.push_back(expanded);
    }
    std::sort(boxes.begin(), boxes.end(), [](const cv::Rect& a, const cv::Rect& b) {
        return a.area() == b.area() ? a.y > b.y : a.area() > b.area();
    });
    if(static_cast<int>(boxes.size()) > limit) boxes.resize(limit);
    return boxes;
}
} // namespace

bool LostItemDetector::load(const std::string& model_path, const std::string& metadata_path)
{
    try
    {
        svm_ = cv::Algorithm::load<cv::ml::SVM>(model_path);
        if(svm_.empty() || !svm_->isTrained()) throw std::runtime_error("SVM 模型无效");
        std::ifstream stream(metadata_path);
        if(!stream) throw std::runtime_error("无法打开模型元数据: " + metadata_path);
        const std::string json((std::istreambuf_iterator<char>(stream)), {});
        raw_sign_ = readNumber(json, "raw_sign", raw_sign_);
        decision_threshold_ = readNumber(json, "decision_threshold", decision_threshold_);
        near_min_aspect_ = readNumber(json, "near_min_aspect", near_min_aspect_);
        near_width_reference_ = readNumber(json, "near_width_reference", near_width_reference_);
        near_threshold_slope_ = readNumber(json, "near_threshold_slope", near_threshold_slope_);
        near_threshold_floor_ = readNumber(json, "near_threshold_floor", near_threshold_floor_);
        cv::HOGDescriptor hog({96, 96}, {16, 16}, {8, 8}, {8, 8}, 9);
        if(svm_->getVarCount() != static_cast<int>(hog.getDescriptorSize()))
            throw std::runtime_error("HOG 特征维度与 SVM 不匹配");
        error_.clear();
        return true;
    }
    catch(const std::exception& exception)
    {
        error_ = exception.what();
        svm_.release();
        return false;
    }
}

cv::Mat LostItemDetector::letterbox(const cv::Mat& image) const
{
    constexpr int size = 96;
    constexpr int content = 88;
    const double scale = std::min(content / static_cast<double>(std::max(1, image.cols)),
                                  content / static_cast<double>(std::max(1, image.rows)));
    cv::Mat resized;
    cv::resize(image, resized,
               {std::max(1, static_cast<int>(std::lround(image.cols * scale))),
                std::max(1, static_cast<int>(std::lround(image.rows * scale)))},
               0, 0, scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    cv::Mat canvas(size, size, CV_8UC3, cv::Scalar(127, 127, 127));
    resized.copyTo(canvas(cv::Rect((size - resized.cols) / 2, (size - resized.rows) / 2,
                                  resized.cols, resized.rows)));
    return canvas;
}

float LostItemDetector::thresholdFor(const cv::Mat& crop) const
{
    const float aspect = crop.cols / static_cast<float>(std::max(1, crop.rows));
    if(aspect < near_min_aspect_) return decision_threshold_;
    const float adjustment = near_threshold_slope_ * std::max(0.0F, crop.cols - near_width_reference_);
    return std::max(near_threshold_floor_, decision_threshold_ - adjustment);
}

LostItemResult LostItemDetector::detect(const cv::Mat& frame, int max_candidates) const
{
    LostItemResult result;
    result.annotated_frame = frame.clone();
    if(svm_.empty()) { result.error = error_.empty() ? "SVM 未加载" : error_; return result; }
    for(const auto& box : candidates(frame, std::max(1, max_candidates)))
    {
        const cv::Mat crop = frame(box);
        auto detection = classifyCrop(crop);
        detection.bounding_box = box;
        if(detection.margin < detection.threshold) continue;
        result.items.push_back(detection);
        cv::rectangle(result.annotated_frame, box, {0, 0, 255}, 3);
        std::ostringstream label;
        label << "LOST ITEM " << std::fixed << std::setprecision(2) << detection.margin;
        cv::putText(result.annotated_frame, label.str(), {box.x, std::max(25, box.y - 8)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 255}, 2, cv::LINE_AA);
    }
    return result;
}

LostItemDetection LostItemDetector::classifyCrop(const cv::Mat& crop) const
{
    cv::HOGDescriptor hog({96, 96}, {16, 16}, {8, 8}, {8, 8}, 9);
    std::vector<float> values;
    hog.compute(letterbox(crop), values);
    cv::Mat feature(1, static_cast<int>(values.size()), CV_32F, values.data());
    const float margin = svm_->predict(feature, cv::noArray(), cv::ml::StatModel::RAW_OUTPUT) * raw_sign_;
    return {{0, 0, crop.cols, crop.rows}, margin, thresholdFor(crop)};
}

std::string LostItemResult::toJson() const
{
    std::ostringstream out;
    out << "{\"has_lost_item\":" << (items.empty() ? "false" : "true") << ",\"items\":[";
    for(std::size_t i = 0; i < items.size(); ++i)
    {
        if(i) out << ',';
        const auto& item = items[i];
        out << "{\"type\":\"target_bundle\",\"margin\":" << item.margin
            << ",\"threshold\":" << item.threshold << ",\"bbox\":["
            << item.bounding_box.x << ',' << item.bounding_box.y << ','
            << item.bounding_box.width << ',' << item.bounding_box.height << "]}";
    }
    out << ']';
    if(!error.empty()) out << ",\"error\":\"" << error << '"';
    out << '}';
    return out.str();
}
} // namespace robot::vision
