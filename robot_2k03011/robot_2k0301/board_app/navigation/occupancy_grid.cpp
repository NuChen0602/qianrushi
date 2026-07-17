#include "navigation/occupancy_grid.h"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace robot::navigation
{
namespace
{
std::string trim(std::string text)
{
    const auto begin = text.find_first_not_of(" \t\r\n");
    if(begin == std::string::npos) return {};
    return text.substr(begin, text.find_last_not_of(" \t\r\n") - begin + 1);
}

std::string token(std::istream& input)
{
    std::string value;
    while(input >> value)
    {
        if(!value.empty() && value.front() == '#') { std::getline(input, value); continue; }
        return value;
    }
    throw std::runtime_error("invalid PGM header");
}
} // namespace

OccupancyGrid::OccupancyGrid(int width, int height, double resolution, double ox,
    double oy, double oyaw, std::vector<std::int8_t> data, int threshold, bool unknown)
    : width_(width), height_(height), resolution_(resolution), origin_x_(ox), origin_y_(oy),
      origin_yaw_(oyaw), occupied_threshold_(threshold), unknown_is_occupied_(unknown),
      data_(std::move(data)), dynamic_(data_.size(), 0)
{
    if(!valid()) throw std::invalid_argument("invalid occupancy grid");
}

OccupancyGrid OccupancyGrid::loadMapYaml(const std::string& yaml_path)
{
    std::ifstream yaml(yaml_path);
    if(!yaml) throw std::runtime_error("cannot open map yaml: " + yaml_path);
    std::string image;
    double resolution = 0.0, ox = 0.0, oy = 0.0, oyaw = 0.0, occupied = 0.65, free = 0.25;
    int negate = 0;
    std::string line;
    while(std::getline(yaml, line))
    {
        const auto split = line.find(':');
        if(split == std::string::npos) continue;
        const std::string key = trim(line.substr(0, split));
        std::string value = trim(line.substr(split + 1));
        if(key == "image") image = value;
        else if(key == "resolution") resolution = std::stod(value);
        else if(key == "negate") negate = std::stoi(value);
        else if(key == "occupied_thresh") occupied = std::stod(value);
        else if(key == "free_thresh") free = std::stod(value);
        else if(key == "origin")
        {
            for(char& ch : value) if(ch == '[' || ch == ']' || ch == ',') ch = ' ';
            std::istringstream values(value); values >> ox >> oy >> oyaw;
        }
    }
    std::string image_path = image;
    if(image.empty()) throw std::runtime_error("map yaml has no image");
    if(image.front() != '/')
    {
        const auto slash = yaml_path.find_last_of("/\\");
        image_path = (slash == std::string::npos ? std::string() : yaml_path.substr(0, slash + 1)) + image;
    }
    std::ifstream pgm(image_path, std::ios::binary);
    if(!pgm) throw std::runtime_error("cannot open map image: " + image_path);
    const std::string magic = token(pgm);
    const int width = std::stoi(token(pgm)), height = std::stoi(token(pgm));
    const int maximum = std::stoi(token(pgm));
    pgm.get();
    std::vector<unsigned char> pixels(static_cast<std::size_t>(width * height));
    if(magic == "P5") pgm.read(reinterpret_cast<char*>(pixels.data()), pixels.size());
    else if(magic == "P2") for(auto& pixel : pixels) pixel = static_cast<unsigned char>(std::stoi(token(pgm)));
    else throw std::runtime_error("unsupported PGM format: " + magic);
    if(!pgm && magic == "P5") throw std::runtime_error("truncated map image");
    std::vector<std::int8_t> data(pixels.size(), -1);
    for(int source_y = 0; source_y < height; ++source_y)
    {
        const int grid_y = height - source_y - 1;
        for(int x = 0; x < width; ++x)
        {
            double probability = pixels[source_y * width + x] / static_cast<double>(maximum);
            if(!negate) probability = 1.0 - probability;
            data[grid_y * width + x] = probability > occupied ? 100 : (probability < free ? 0 : -1);
        }
    }
    return {width, height, resolution, ox, oy, oyaw, std::move(data)};
}

bool OccupancyGrid::valid() const
{ return width_ > 0 && height_ > 0 && resolution_ > 0.0 && data_.size() == static_cast<std::size_t>(width_ * height_); }
bool OccupancyGrid::contains(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }
bool OccupancyGrid::occupiedCell(int x, int y) const
{
    if(!contains(x, y)) return true;
    const auto index = static_cast<std::size_t>(y * width_ + x);
    if(dynamic_[index]) return true;
    return data_[index] < 0 ? unknown_is_occupied_ : data_[index] >= occupied_threshold_;
}
bool OccupancyGrid::unknownCell(int x, int y) const
{
    return contains(x, y) && data_[static_cast<std::size_t>(y * width_ + x)] < 0;
}
std::pair<int, int> OccupancyGrid::worldToGrid(double x, double y) const
{
    const double dx = x - origin_x_, dy = y - origin_y_;
    const double local_x = std::cos(origin_yaw_) * dx + std::sin(origin_yaw_) * dy;
    const double local_y = -std::sin(origin_yaw_) * dx + std::cos(origin_yaw_) * dy;
    return {static_cast<int>(std::floor(local_x / resolution_)), static_cast<int>(std::floor(local_y / resolution_))};
}
std::pair<double, double> OccupancyGrid::gridToWorld(int x, int y) const
{
    const double lx = (x + 0.5) * resolution_, ly = (y + 0.5) * resolution_;
    return {origin_x_ + std::cos(origin_yaw_) * lx - std::sin(origin_yaw_) * ly,
            origin_y_ + std::sin(origin_yaw_) * lx + std::cos(origin_yaw_) * ly};
}
bool OccupancyGrid::occupiedWorld(double x, double y) const { const auto cell = worldToGrid(x, y); return occupiedCell(cell.first, cell.second); }
bool OccupancyGrid::occupiedNearWorld(double x, double y, double radius) const
{
    const auto center = worldToGrid(x, y);
    const int cells = static_cast<int>(std::ceil(radius / resolution_));
    for(int gy = center.second - cells; gy <= center.second + cells; ++gy)
        for(int gx = center.first - cells; gx <= center.first + cells; ++gx)
        {
            if(!occupiedCell(gx, gy)) continue;
            const auto world = gridToWorld(gx, gy);
            if(std::hypot(world.first - x, world.second - y) <= radius + resolution_ * 0.71) return true;
        }
    return false;
}
void OccupancyGrid::clearDynamicObstacles() { std::fill(dynamic_.begin(), dynamic_.end(), 0); }
void OccupancyGrid::setDynamicObstacles(const std::vector<LaserPoint>& points, double radius)
{
    clearDynamicObstacles();
    const int cells = std::max(0, static_cast<int>(std::ceil(radius / resolution_)));
    for(const auto& point : points)
    {
        const auto center = worldToGrid(point.x, point.y);
        for(int y = center.second - cells; y <= center.second + cells; ++y)
            for(int x = center.first - cells; x <= center.first + cells; ++x)
                if(contains(x, y) && std::hypot(x - center.first, y - center.second) * resolution_ <= radius)
                    dynamic_[static_cast<std::size_t>(y * width_ + x)] = 1;
    }
}
} // namespace robot::navigation
