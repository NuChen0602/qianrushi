#include "vision/lost_item_detector.h"
#include "vision/detection_tracker.h"
#if ROBOT_HAVE_OPENCV_ARUCO
#include "vision/aruco_book_detector.h"
#endif

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
std::atomic_bool running{true};
void stop(int) { running = false; }

struct Options
{
    std::string camera = "/dev/video0";
    std::string image;
    std::string output = "/tmp/book_detection.jpg";
    std::string mode = "book";
    std::string model = "models/lost_item_hog_svm.xml";
    std::string metadata = "models/lost_item_hog_svm.json";
    int expected_id = 203;
    int width = 640;
    int height = 480;
    int interval_ms = 250;
    int stream_port = 0;
    int jpeg_quality = 80;
    int max_frames = 0;
    bool once = false;
    bool require_detection = false;
};

class JpegStreamServer
{
public:
    ~JpegStreamServer()
    {
        closeClient();
        if(server_fd_ >= 0) ::close(server_fd_);
    }

    bool open(int port)
    {
        if(port <= 0) return true;
        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if(server_fd_ < 0) return false;
        int reuse = 1;
        ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_ANY);
        address.sin_port = htons(static_cast<std::uint16_t>(port));
        if(::bind(server_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 ||
           ::listen(server_fd_, 1) != 0)
        {
            ::close(server_fd_);
            server_fd_ = -1;
            return false;
        }
        setNonBlocking(server_fd_);
        std::cerr << "JPEG stream listening on 0.0.0.0:" << port << '\n';
        return true;
    }

    void publish(const cv::Mat& image, int quality)
    {
        acceptClient();
        if(client_fd_ < 0 || image.empty()) return;
        flush();
        if(!pending_.empty()) return; // 慢客户端只丢新帧，不阻塞视觉处理。

        std::vector<unsigned char> jpeg;
        if(!cv::imencode(".jpg", image, jpeg,
                        {cv::IMWRITE_JPEG_QUALITY, std::max(1, std::min(100, quality))})) return;
        const auto size = htonl(static_cast<std::uint32_t>(jpeg.size()));
        pending_.resize(sizeof(size) + jpeg.size());
        std::memcpy(pending_.data(), &size, sizeof(size));
        std::memcpy(pending_.data() + sizeof(size), jpeg.data(), jpeg.size());
        offset_ = 0;
        flush();
    }

private:
    static void setNonBlocking(int fd)
    {
        const int flags = ::fcntl(fd, F_GETFL, 0);
        if(flags >= 0) ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    void acceptClient()
    {
        if(server_fd_ < 0) return;
        const int fd = ::accept(server_fd_, nullptr, nullptr);
        if(fd < 0) return;
        setNonBlocking(fd);
        closeClient();
        client_fd_ = fd;
        std::cerr << "JPEG stream client connected\n";
    }

    void flush()
    {
        while(client_fd_ >= 0 && offset_ < pending_.size())
        {
            const auto written = ::send(client_fd_, pending_.data() + offset_,
                                        pending_.size() - offset_, MSG_NOSIGNAL);
            if(written > 0) offset_ += static_cast<std::size_t>(written);
            else if(written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
            else { closeClient(); return; }
        }
        if(offset_ == pending_.size())
        {
            pending_.clear();
            offset_ = 0;
        }
    }

    void closeClient()
    {
        if(client_fd_ >= 0) ::close(client_fd_);
        client_fd_ = -1;
        pending_.clear();
        offset_ = 0;
    }

    int server_fd_ = -1;
    int client_fd_ = -1;
    std::vector<unsigned char> pending_;
    std::size_t offset_ = 0;
};

void usage()
{
    std::cout << "Usage:\n"
              << "  robot_board_vision --camera /dev/video0 --expected-id 203\n"
              << "  robot_board_vision --image test.jpg --expected-id 203 --output result.jpg\n"
              << "  robot_board_vision --mode lost-item --camera /dev/video0\n"
              << "Options: --mode book|lost-item|lost-item-crop --model PATH --metadata PATH --once\n"
              << "         --width N --height N --interval-ms N --output PATH\n"
              << "         --stream-port N --jpeg-quality 1..100 --max-frames N\n"
              << "         --require-detection (exit 3 when no detection is confirmed)\n";
}

bool value(int& index, int argc, char** argv, std::string& out)
{
    if(index + 1 >= argc) return false;
    out = argv[++index];
    return true;
}

bool parse(int argc, char** argv, Options& options)
{
    for(int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        std::string text;
        if(arg == "--help" || arg == "-h") { usage(); return false; }
        if(arg == "--once") { options.once = true; continue; }
        if(arg == "--require-detection") { options.require_detection = true; continue; }
        if(!value(i, argc, argv, text)) return false;
        try
        {
            if(arg == "--camera") options.camera = text;
            else if(arg == "--image") options.image = text;
            else if(arg == "--output") options.output = text;
            else if(arg == "--mode") options.mode = text;
            else if(arg == "--model") options.model = text;
            else if(arg == "--metadata") options.metadata = text;
            else if(arg == "--expected-id") options.expected_id = std::stoi(text);
            else if(arg == "--width") options.width = std::stoi(text);
            else if(arg == "--height") options.height = std::stoi(text);
            else if(arg == "--interval-ms") options.interval_ms = std::stoi(text);
            else if(arg == "--stream-port") options.stream_port = std::stoi(text);
            else if(arg == "--jpeg-quality") options.jpeg_quality = std::stoi(text);
            else if(arg == "--max-frames") options.max_frames = std::stoi(text);
            else return false;
        }
        catch(const std::exception&) { return false; }
    }
    return true;
}

int process(const cv::Mat& frame, const Options& options,
#if ROBOT_HAVE_OPENCV_ARUCO
            const robot::vision::ArucoBookDetector& book_detector,
#endif
            const robot::vision::LostItemDetector& lost_item_detector,
            robot::vision::DetectionTracker& tracker,
            cv::Mat* annotated_output = nullptr,
            bool* detected_output = nullptr)
{
    cv::Mat annotated;
    bool detected = false;
    if(options.mode == "lost-item-crop")
    {
        const auto item = lost_item_detector.classifyCrop(frame);
        const bool found = item.margin >= item.threshold;
        detected = found;
        std::cout << "{\"has_lost_item\":" << (found ? "true" : "false")
                  << ",\"margin\":" << item.margin
                  << ",\"threshold\":" << item.threshold << "}" << std::endl;
        annotated = frame.clone();
    }
    else if(options.mode == "lost-item")
    {
        const auto result = lost_item_detector.detect(frame);
        std::vector<cv::Rect> boxes;
        for(const auto& item : result.items) boxes.push_back(item.bounding_box);
        const auto& tracks = tracker.update(boxes);
        annotated = frame.clone();
        bool confirmed = false;
        std::cout << "{\"has_lost_item\":";
        for(const auto& track : tracks) if(track.confirmed) confirmed = true;
        detected = confirmed;
        std::cout << (confirmed ? "true" : "false") << ",\"tracks\":[";
        for(std::size_t index = 0; index < tracks.size(); ++index)
        {
            const auto& track = tracks[index];
            if(index) std::cout << ',';
            std::cout << "{\"track_id\":" << track.track_id
                      << ",\"confirmed\":" << (track.confirmed ? "true" : "false")
                      << ",\"positive_frames\":" << track.positive_frames
                      << ",\"missed_frames\":" << track.missed_frames
                      << ",\"bbox\":[" << track.box.x << ',' << track.box.y << ','
                      << track.box.width << ',' << track.box.height << "]}";
            const cv::Scalar color = track.confirmed ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 255);
            cv::rectangle(annotated, track.box, color, track.confirmed ? 3 : 2);
            cv::putText(annotated, "TRACK " + std::to_string(track.track_id),
                        {track.box.x, std::max(25, track.box.y - 8)},
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv::LINE_AA);
        }
        std::cout << "]}" << std::endl;
    }
    else
    {
#if ROBOT_HAVE_OPENCV_ARUCO
        const auto result = book_detector.detect(frame);
        detected = result.found;
        std::cout << result.toJson() << std::endl;
        annotated = result.annotated_frame;
#else
        std::cerr << "book mode unavailable: board OpenCV was built without aruco\n";
        return 1;
#endif
    }
    if(!options.output.empty() && !annotated.empty() &&
       !cv::imwrite(options.output, annotated))
    {
        std::cerr << "cannot write annotated image: " << options.output << '\n';
        return 1;
    }
    if(annotated_output != nullptr) *annotated_output = annotated;
    if(detected_output != nullptr) *detected_output = detected;
    return 0;
}
} // namespace

int main(int argc, char** argv)
{
    if(argc == 2 && std::string(argv[1]) == "--tracker-self-test")
    {
        robot::vision::DetectionTracker tracker(3, 4, 0.12);
        const std::vector<cv::Rect> detection = {{10, 10, 40, 30}};
        tracker.update(detection);
        tracker.update({{12, 10, 40, 30}});
        const auto& tracks = tracker.update({{14, 11, 40, 30}});
        const bool passed = tracks.size() == 1 && tracks[0].confirmed && tracks[0].track_id == 1;
        std::cout << (passed ? "PASS" : "FAIL") << " detection tracker\n";
        return passed ? 0 : 1;
    }
    Options options;
    if(!parse(argc, argv, options))
    {
        if(argc > 1 && std::string(argv[1]) != "--help" && std::string(argv[1]) != "-h") usage();
        return argc > 1 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") ? 0 : 2;
    }
    if(options.mode != "book" && options.mode != "lost-item" && options.mode != "lost-item-crop")
    {
        std::cerr << "unsupported mode: " << options.mode << '\n';
        return 2;
    }
#if ROBOT_HAVE_OPENCV_ARUCO
    const robot::vision::ArucoBookDetector book_detector(options.expected_id);
#endif
    robot::vision::LostItemDetector lost_item_detector;
    robot::vision::DetectionTracker tracker(3, 4, 0.12);
    if(options.mode != "book" && !lost_item_detector.load(options.model, options.metadata))
    {
        std::cerr << "cannot load lost-item model: " << lost_item_detector.error() << '\n';
        return 1;
    }
    if(!options.image.empty())
    {
        const auto image = cv::imread(options.image, cv::IMREAD_COLOR);
        if(image.empty()) { std::cerr << "cannot read image: " << options.image << '\n'; return 1; }
        bool detected = false;
        const int result = process(image, options,
#if ROBOT_HAVE_OPENCV_ARUCO
                       book_detector,
#endif
                       lost_item_detector, tracker, nullptr, &detected);
        if(result != 0) return result;
        return options.require_detection && !detected ? 3 : 0;
    }

    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    cv::VideoCapture camera(options.camera, cv::CAP_V4L2);
    if(!camera.isOpened()) { std::cerr << "cannot open camera: " << options.camera << '\n'; return 1; }
    JpegStreamServer stream;
    if(!stream.open(options.stream_port))
    {
        std::cerr << "cannot open JPEG stream port: " << options.stream_port << '\n';
        return 1;
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
    int processed_frames = 0;
    bool detected = false;
    while(running)
    {
        cv::Mat frame;
        if(!camera.read(frame) || frame.empty())
        {
            std::cerr << "camera frame unavailable\n";
            return 1;
        }
        cv::Mat annotated;
        bool frame_detected = false;
        if(process(frame, options,
#if ROBOT_HAVE_OPENCV_ARUCO
                   book_detector,
#endif
                   lost_item_detector, tracker, &annotated, &frame_detected) != 0) return 1;
        stream.publish(annotated, options.jpeg_quality);
        detected = detected || frame_detected;
        ++processed_frames;
        if(options.require_detection && detected) break;
        if(options.max_frames > 0 && processed_frames >= options.max_frames) break;
        if(options.once) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(std::max(1, options.interval_ms)));
    }
    return options.require_detection && !detected ? 3 : 0;
}
