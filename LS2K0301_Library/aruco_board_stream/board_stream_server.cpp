#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {

struct Options {
    std::string camera = "/dev/video0";
    int width = 320;
    int height = 240;
    int fps = 15;
    int port = 5000;
    int jpeg_quality = 80;
    int rotate = 0;
    std::string framebuffer;
};

void printUsage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --camera <device>       Camera device (default /dev/video0)\n"
        << "  --width <pixels>        Output width (default 320)\n"
        << "  --height <pixels>       Output height (default 240)\n"
        << "  --fps <value>           Requested camera FPS (default 15)\n"
        << "  --port <value>          TCP listen port (default 5000)\n"
        << "  --jpeg-quality <1-100>  JPEG quality (default 80)\n"
        << "  --rotate <0|90|180|270> Output rotation (default 0)\n"
        << "  --framebuffer <device>  Also display frames on RGB565 framebuffer\n"
        << "  --help                  Show this help\n";
}

int parseInt(const std::string& value, const std::string& name) {
    std::size_t parsed = 0;
    int result = 0;
    try {
        result = std::stoi(value, &parsed);
    } catch (const std::exception&) {
        throw std::runtime_error("Invalid integer for " + name + ": " + value);
    }
    if (parsed != value.size()) {
        throw std::runtime_error("Invalid integer for " + name + ": " + value);
    }
    return result;
}

Options parseArgs(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (i + 1 >= argc) {
            throw std::runtime_error("Missing value for " + arg);
        }
        const std::string value = argv[++i];
        if (arg == "--camera") {
            options.camera = value;
        } else if (arg == "--width") {
            options.width = parseInt(value, arg);
        } else if (arg == "--height") {
            options.height = parseInt(value, arg);
        } else if (arg == "--fps") {
            options.fps = parseInt(value, arg);
        } else if (arg == "--port") {
            options.port = parseInt(value, arg);
        } else if (arg == "--jpeg-quality") {
            options.jpeg_quality = parseInt(value, arg);
        } else if (arg == "--rotate") {
            options.rotate = parseInt(value, arg);
        } else if (arg == "--framebuffer") {
            options.framebuffer = value;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.width <= 0 || options.height <= 0 || options.fps <= 0) {
        throw std::runtime_error("width, height and fps must be positive");
    }
    if (options.port < 1 || options.port > 65535) {
        throw std::runtime_error("port must be in range 1..65535");
    }
    if (options.jpeg_quality < 1 || options.jpeg_quality > 100) {
        throw std::runtime_error("jpeg-quality must be in range 1..100");
    }
    if (options.rotate != 0 && options.rotate != 90 &&
        options.rotate != 180 && options.rotate != 270) {
        throw std::runtime_error("rotate must be one of 0, 90, 180 or 270");
    }
    return options;
}

class FramebufferDisplay {
public:
    explicit FramebufferDisplay(const std::string& device) {
        if (device.empty()) {
            return;
        }
        fd_ = ::open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            throw std::runtime_error("open " + device + " failed: " + std::strerror(errno));
        }
        if (::ioctl(fd_, FBIOGET_FSCREENINFO, &fixed_) < 0 ||
            ::ioctl(fd_, FBIOGET_VSCREENINFO, &variable_) < 0) {
            const std::string error = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("query framebuffer failed: " + error);
        }
        if (variable_.bits_per_pixel != 16) {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("framebuffer must use 16-bit RGB565");
        }
        size_ = static_cast<std::size_t>(fixed_.line_length) * variable_.yres_virtual;
        memory_ = static_cast<std::uint8_t*>(
            ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
        if (memory_ == MAP_FAILED) {
            memory_ = nullptr;
            const std::string error = std::strerror(errno);
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("mmap framebuffer failed: " + error);
        }
        std::memset(memory_, 0, size_);
        std::cout << "Framebuffer: " << device << ' ' << variable_.xres << 'x'
                  << variable_.yres << " RGB565" << std::endl;
    }

    ~FramebufferDisplay() {
        if (memory_) {
            ::munmap(memory_, size_);
        }
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    FramebufferDisplay(const FramebufferDisplay&) = delete;
    FramebufferDisplay& operator=(const FramebufferDisplay&) = delete;

    void show(const cv::Mat& bgr) {
        if (!memory_ || bgr.empty()) {
            return;
        }
        const double scale = std::min(
            static_cast<double>(variable_.xres) / bgr.cols,
            static_cast<double>(variable_.yres) / bgr.rows);
        const int width = std::max(1, static_cast<int>(bgr.cols * scale));
        const int height = std::max(1, static_cast<int>(bgr.rows * scale));
        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
        const int offset_x = (static_cast<int>(variable_.xres) - width) / 2;
        const int offset_y = (static_cast<int>(variable_.yres) - height) / 2;

        for (int y = 0; y < height; ++y) {
            auto* destination = reinterpret_cast<std::uint16_t*>(
                memory_ + (offset_y + y) * fixed_.line_length) + offset_x;
            const auto* source = resized.ptr<cv::Vec3b>(y);
            for (int x = 0; x < width; ++x) {
                const std::uint16_t blue = static_cast<std::uint16_t>(source[x][0] >> 3);
                const std::uint16_t green = static_cast<std::uint16_t>(source[x][1] >> 2);
                const std::uint16_t red = static_cast<std::uint16_t>(source[x][2] >> 3);
                destination[x] = static_cast<std::uint16_t>((red << 11) | (green << 5) | blue);
            }
        }
    }

private:
    int fd_ = -1;
    std::size_t size_ = 0;
    std::uint8_t* memory_ = nullptr;
    fb_fix_screeninfo fixed_{};
    fb_var_screeninfo variable_{};
};

void rotateFrame(const cv::Mat& input, cv::Mat& output, int angle) {
    if (angle == 90) {
        cv::rotate(input, output, cv::ROTATE_90_CLOCKWISE);
    } else if (angle == 180) {
        cv::rotate(input, output, cv::ROTATE_180);
    } else if (angle == 270) {
        cv::rotate(input, output, cv::ROTATE_90_COUNTERCLOCKWISE);
    } else {
        output = input;
    }
}

bool sendAll(int socket_fd, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::size_t sent = 0;
    while (sent < size) {
        const ssize_t result = ::send(socket_fd, bytes + sent, size - sent, 0);
        if (result > 0) {
            sent += static_cast<std::size_t>(result);
            continue;
        }
        if (result < 0 && errno == EINTR) {
            continue;
        }
        return false;
    }
    return true;
}

int createServerSocket(int port) {
    const int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        throw std::runtime_error("socket failed: " + std::string(std::strerror(errno)));
    }

    int reuse = 1;
    if (::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        ::close(server_fd);
        throw std::runtime_error("setsockopt failed: " + std::string(std::strerror(errno)));
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        ::close(server_fd);
        throw std::runtime_error("bind failed: " + std::string(std::strerror(errno)));
    }
    if (::listen(server_fd, 1) < 0) {
        ::close(server_fd);
        throw std::runtime_error("listen failed: " + std::string(std::strerror(errno)));
    }
    return server_fd;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArgs(argc, argv);
        ::signal(SIGPIPE, SIG_IGN);
        FramebufferDisplay display(options.framebuffer);

        cv::VideoCapture camera;
        if (!camera.open(options.camera, cv::CAP_V4L2)) {
            std::cerr << "Failed to open camera " << options.camera << " with V4L2\n";
            return 1;
        }
        camera.set(cv::CAP_PROP_FRAME_WIDTH, options.width);
        camera.set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
        camera.set(cv::CAP_PROP_FPS, options.fps);

        std::cout << "Camera: " << options.camera << '\n'
                  << "Requested capture: " << options.width << 'x' << options.height
                  << " @ " << options.fps << " FPS\n"
                  << "Actual capture: "
                  << static_cast<int>(camera.get(cv::CAP_PROP_FRAME_WIDTH)) << 'x'
                  << static_cast<int>(camera.get(cv::CAP_PROP_FRAME_HEIGHT)) << " @ "
                  << camera.get(cv::CAP_PROP_FPS) << " FPS\n"
                  << "JPEG quality: " << options.jpeg_quality
                  << ", rotate: " << options.rotate << '\n';

        const int server_fd = createServerSocket(options.port);
        std::cout << "Listening on 0.0.0.0:" << options.port << std::endl;

        const std::vector<int> encode_params = {
            cv::IMWRITE_JPEG_QUALITY, options.jpeg_quality};
        while (true) {
            sockaddr_in client_address{};
            socklen_t client_length = sizeof(client_address);
            const int client_fd = ::accept(
                server_fd, reinterpret_cast<sockaddr*>(&client_address), &client_length);
            if (client_fd < 0) {
                if (errno == EINTR) {
                    continue;
                }
                std::cerr << "accept failed: " << std::strerror(errno) << '\n';
                continue;
            }

            char client_ip[INET_ADDRSTRLEN] = {};
            ::inet_ntop(AF_INET, &client_address.sin_addr, client_ip, sizeof(client_ip));
            std::cout << "Client connected: " << client_ip << ':'
                      << ntohs(client_address.sin_port) << std::endl;

            std::uint64_t frame_count = 0;
            std::uint64_t byte_count = 0;
            auto report_time = std::chrono::steady_clock::now();

            while (true) {
                cv::Mat captured;
                if (!camera.read(captured) || captured.empty()) {
                    std::cerr << "Camera frame read failed\n";
                    break;
                }

                cv::Mat resized;
                cv::resize(captured, resized, cv::Size(options.width, options.height));
                cv::Mat output;
                rotateFrame(resized, output, options.rotate);
                display.show(output);

                std::vector<std::uint8_t> jpeg;
                if (!cv::imencode(".jpg", output, jpeg, encode_params)) {
                    std::cerr << "JPEG encoding failed\n";
                    continue;
                }
                if (jpeg.size() > UINT32_MAX) {
                    std::cerr << "JPEG frame is too large\n";
                    continue;
                }

                const std::uint32_t network_size =
                    htonl(static_cast<std::uint32_t>(jpeg.size()));
                if (!sendAll(client_fd, &network_size, sizeof(network_size)) ||
                    !sendAll(client_fd, jpeg.data(), jpeg.size())) {
                    std::cout << "Client disconnected" << std::endl;
                    break;
                }

                ++frame_count;
                byte_count += jpeg.size();
                const auto now = std::chrono::steady_clock::now();
                const double elapsed =
                    std::chrono::duration<double>(now - report_time).count();
                if (elapsed >= 1.0) {
                    const double stream_fps = static_cast<double>(frame_count) / elapsed;
                    const double average_kib = frame_count == 0
                        ? 0.0
                        : static_cast<double>(byte_count) / frame_count / 1024.0;
                    std::cout << "stream_fps=" << stream_fps
                              << " avg_jpeg=" << average_kib << " KiB" << std::endl;
                    frame_count = 0;
                    byte_count = 0;
                    report_time = now;
                }
            }
            ::close(client_fd);
            std::cout << "Waiting for next client on port " << options.port << std::endl;
        }
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
