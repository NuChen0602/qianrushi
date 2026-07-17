#pragma once

#include <opencv2/core/mat.hpp>

#include <string>
#include <vector>

namespace robot::vision
{
struct BookDetection
{
    int id = -1;
    std::string name;
    int rank = 0;
    int visual_rank = 0;
    cv::Rect bounding_box;
    double center_x = 0.0;
};

struct BookDetectionResult
{
    bool found = false;
    int expected_id = -1;
    std::vector<BookDetection> books;
    cv::Mat annotated_frame;
    std::string message;

    std::string toJson() const;
};

class ArucoBookDetector
{
public:
    explicit ArucoBookDetector(int expected_id = 203);
    BookDetectionResult detect(const cv::Mat& frame) const;

private:
    int expected_id_;
};
} // namespace robot::vision
