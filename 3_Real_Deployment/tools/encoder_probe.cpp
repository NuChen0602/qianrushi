#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace {

volatile sig_atomic_t g_running = 1;

void handle_signal(int) {
    g_running = 0;
}

bool read_count(int fd, int16_t *value) {
    if (value == nullptr) {
        return false;
    }
    int16_t raw = 0;
    const ssize_t n = read(fd, &raw, sizeof(raw));
    if (n != static_cast<ssize_t>(sizeof(raw))) {
        return false;
    }
    *value = raw;
    return true;
}

bool clear_count(int fd) {
    const int16_t zero = 0;
    const ssize_t n = write(fd, &zero, sizeof(zero));
    return n == static_cast<ssize_t>(sizeof(zero));
}

int open_encoder(const char *path) {
    const int fd = open(path, O_RDWR);
    if (fd < 0) {
        std::printf("open %s failed: %s\n", path, std::strerror(errno));
    }
    return fd;
}

}  // namespace

int main(int argc, char **argv) {
    const int duration_s = argc > 1 ? std::atoi(argv[1]) : 30;
    const int sample_ms = argc > 2 ? std::atoi(argv[2]) : 50;
    const int total_samples = (duration_s * 1000) / sample_ms;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const int left_fd = open_encoder("/dev/zf_encoder_quad_1");
    const int right_fd = open_encoder("/dev/zf_encoder_quad_2");
    if (left_fd < 0 || right_fd < 0) {
        if (left_fd >= 0) close(left_fd);
        if (right_fd >= 0) close(right_fd);
        return 1;
    }

    clear_count(left_fd);
    clear_count(right_fd);

    long left_total = 0;
    long right_total = 0;

    std::printf("encoder_probe duration=%ds sample=%dms\n", duration_s, sample_ms);
    std::printf("Turn one wheel at a time. Ctrl+C to stop.\n");
    std::printf("sample,left_delta,right_delta,left_total,right_total\n");

    for (int i = 0; g_running && i < total_samples; ++i) {
        usleep(sample_ms * 1000);

        int16_t left_delta = 0;
        int16_t right_delta = 0;
        const bool left_ok = read_count(left_fd, &left_delta);
        const bool right_ok = read_count(right_fd, &right_delta);
        clear_count(left_fd);
        clear_count(right_fd);

        if (!left_ok || !right_ok) {
            std::printf("read failed at sample %d: left_ok=%d right_ok=%d\n",
                        i, left_ok ? 1 : 0, right_ok ? 1 : 0);
            break;
        }

        left_total += left_delta;
        right_total += right_delta;
        std::printf("%d,%d,%d,%ld,%ld\n",
                    i, static_cast<int>(left_delta), static_cast<int>(right_delta),
                    left_total, right_total);
        std::fflush(stdout);
    }

    clear_count(left_fd);
    clear_count(right_fd);
    close(left_fd);
    close(right_fd);
    return 0;
}
