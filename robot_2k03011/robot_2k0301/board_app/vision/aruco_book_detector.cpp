#include "vision/aruco_book_detector.h"

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define ROBOT_USE_OFFICIAL_ARUCO 1
#else
#define ROBOT_USE_OFFICIAL_ARUCO 0
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>

namespace robot::vision
{
namespace
{
const std::map<int, std::string> kBookNames = {
    {101, "工程控制论"}, {201, "苏东坡传"}, {202, "霍乱时期的爱情"},
    {203, "百年孤独"}, {204, "瓦尔登湖"}, {205, "大国崛起"},
};
const std::map<int, int> kFixedRanks = {
    {201, 1}, {202, 2}, {203, 3}, {204, 4}, {205, 5},
};
#if !ROBOT_USE_OFFICIAL_ARUCO
// Compatibility decoder for OpenCV releases that predate ArUco in objdetect.
const std::map<int, std::uint32_t> kMarkerBits = {
    {101, 0x126df54U}, {201, 0x1d0f099U}, {202, 0x1641b99U},
    {203, 0x170fa27U}, {204, 0x17608b8U}, {205, 0x10c8354U},
};
int hamming(std::uint32_t value)
{
    int count = 0;
    while(value) { value &= value - 1; ++count; }
    return count;
}

std::uint32_t rotate90(std::uint32_t input)
{
    std::uint32_t output = 0;
    for(int row = 0; row < 5; ++row)
        for(int column = 0; column < 5; ++column)
            if(input & (1U << (row * 5 + column)))
                output |= 1U << (column * 5 + (4 - row));
    return output;
}

std::array<cv::Point2f, 4> orderedCorners(const std::vector<cv::Point>& polygon)
{
    std::array<cv::Point2f, 4> ordered{};
    auto min_sum = polygon.front(), max_sum = polygon.front();
    auto min_diff = polygon.front(), max_diff = polygon.front();
    for(const auto& point : polygon)
    {
        if(point.x + point.y < min_sum.x + min_sum.y) min_sum = point;
        if(point.x + point.y > max_sum.x + max_sum.y) max_sum = point;
        if(point.x - point.y < min_diff.x - min_diff.y) min_diff = point;
        if(point.x - point.y > max_diff.x - max_diff.y) max_diff = point;
    }
    ordered[0] = min_sum;   // top-left
    ordered[1] = max_diff;  // top-right
    ordered[2] = max_sum;   // bottom-right
    ordered[3] = min_diff;  // bottom-left
    return ordered;
}

int decodeMarker(const cv::Mat& gray, const std::array<cv::Point2f, 4>& corners)
{
    constexpr int cell = 10;
    constexpr int cells = 7;
    const std::array<cv::Point2f, 4> destination = {
        cv::Point2f(0, 0), cv::Point2f(cells * cell - 1, 0),
        cv::Point2f(cells * cell - 1, cells * cell - 1), cv::Point2f(0, cells * cell - 1),
    };
    cv::Mat warped, binary;
    cv::warpPerspective(gray, warped, cv::getPerspectiveTransform(corners.data(), destination.data()),
                        {cells * cell, cells * cell}, cv::INTER_LINEAR);
    cv::threshold(warped, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    int white_border_cells = 0;
    for(int row = 0; row < cells; ++row)
        for(int column = 0; column < cells; ++column)
            if((row == 0 || row == cells - 1 || column == 0 || column == cells - 1) &&
               cv::mean(binary(cv::Rect(column * cell + 2, row * cell + 2, cell - 4, cell - 4)))[0] > 127)
                ++white_border_cells;
    if(white_border_cells > 2) return -1;

    std::uint32_t bits = 0;
    for(int row = 0; row < 5; ++row)
        for(int column = 0; column < 5; ++column)
            if(cv::mean(binary(cv::Rect((column + 1) * cell + 2, (row + 1) * cell + 2,
                                        cell - 4, cell - 4)))[0] > 127)
                bits |= 1U << (row * 5 + column);

    int best_id = -1;
    int best_distance = 26;
    for(int rotation = 0; rotation < 4; ++rotation)
    {
        for(const auto& [id, pattern] : kMarkerBits)
        {
            const int distance = hamming(bits ^ pattern);
            if(distance < best_distance) { best_distance = distance; best_id = id; }
        }
        bits = rotate90(bits);
    }
    return best_distance <= 3 ? best_id : -1;
}
#endif

std::string bookName(int id)
{
    const auto it = kBookNames.find(id);
    return it == kBookNames.end() ? "ID" + std::to_string(id) : it->second;
}

std::string jsonEscape(const std::string& value)
{
    std::ostringstream out;
    for(const unsigned char ch : value)
    {
        switch(ch)
        {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if(ch < 0x20) out << "\\u" << std::hex << std::setw(4)
                                  << std::setfill('0') << static_cast<int>(ch);
                else out << ch;
        }
    }
    return out.str();
}
} // namespace

ArucoBookDetector::ArucoBookDetector(int expected_id) : expected_id_(expected_id) {}

BookDetectionResult ArucoBookDetector::detect(const cv::Mat& frame) const
{
    BookDetectionResult result;
    result.expected_id = expected_id_;
    if(frame.empty()) { result.message = "摄像头图像为空"; return result; }
    result.annotated_frame = frame.clone();

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
#if ROBOT_USE_OFFICIAL_ARUCO
    const cv::aruco::ArucoDetector detector(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250));
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    detector.detectMarkers(gray, marker_corners, marker_ids);
    for(std::size_t index = 0; index < marker_ids.size(); ++index)
    {
        const int id = marker_ids[index];
        if(kBookNames.find(id) == kBookNames.end()) continue;
        const cv::Rect box = cv::boundingRect(marker_corners[index]);
        result.books.push_back({id, bookName(id), 0, 0, box, box.x + box.width * 0.5});
    }
#else
    cv::Mat thresholded;
    cv::adaptiveThreshold(gray, thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY_INV, 21, 7);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresholded, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    for(const auto& contour : contours)
    {
        const double perimeter = cv::arcLength(contour, true);
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, perimeter * 0.04, true);
        if(polygon.size() != 4 || !cv::isContourConvex(polygon) || std::abs(cv::contourArea(polygon)) < 400)
            continue;
        const auto corners = orderedCorners(polygon);
        const int id = decodeMarker(gray, corners);
        if(id < 0) continue;
        const cv::Rect box = cv::boundingRect(polygon);
        bool duplicate = false;
        for(const auto& book : result.books)
            if(book.id == id && (book.bounding_box & box).area() > box.area() * 0.5) duplicate = true;
        if(!duplicate) result.books.push_back({id, bookName(id), 0, 0, box, box.x + box.width * 0.5});
    }
#endif

    std::sort(result.books.begin(), result.books.end(),
              [](const BookDetection& a, const BookDetection& b) { return a.center_x < b.center_x; });
    const BookDetection* target = nullptr;
    for(std::size_t index = 0; index < result.books.size(); ++index)
    {
        auto& book = result.books[index];
        book.visual_rank = static_cast<int>(index + 1);
        const auto rank = kFixedRanks.find(book.id);
        book.rank = rank == kFixedRanks.end() ? book.visual_rank : rank->second;
        if(book.id == expected_id_) target = &book;
        const bool expected = book.id == expected_id_;
        const cv::Scalar color = expected ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 180, 0);
        cv::rectangle(result.annotated_frame, book.bounding_box, color, expected ? 3 : 2);
        std::string label = "ID" + std::to_string(book.id) + " R" + std::to_string(book.rank);
        if(expected) label += " TARGET";
        cv::putText(result.annotated_frame, label,
                    {book.bounding_box.x, std::max(25, book.bounding_box.y - 8)},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
    }
    result.found = target != nullptr;
    if(target) result.message = "已识别到《" + target->name + "》，从左往右第" +
                                std::to_string(target->rank) + "本";
    else if(!result.books.empty()) result.message = "已识别到书籍标签，但未找到目标书籍";
    else result.message = "未识别到书籍标签";
    return result;
}

std::string BookDetectionResult::toJson() const
{
    std::ostringstream out;
    out << "{\"found\":" << (found ? "true" : "false")
        << ",\"expected_id\":" << expected_id << ",\"books\":[";
    for(std::size_t index = 0; index < books.size(); ++index)
    {
        const auto& book = books[index];
        if(index) out << ',';
        out << "{\"id\":" << book.id << ",\"name\":\"" << jsonEscape(book.name) << "\""
            << ",\"rank\":" << book.rank << ",\"visual_rank\":" << book.visual_rank
            << ",\"bbox\":[" << book.bounding_box.x << ',' << book.bounding_box.y << ','
            << book.bounding_box.x + book.bounding_box.width << ','
            << book.bounding_box.y + book.bounding_box.height << "]}";
    }
    out << "],\"message\":\"" << jsonEscape(message) << "\"}";
    return out.str();
}
} // namespace robot::vision
