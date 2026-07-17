#pragma once

#include "navigation/navigation_types.h"

#include <string>

namespace robot::navigation
{
class OccupancyGrid
{
public:
    OccupancyGrid() = default;
    OccupancyGrid(int width, int height, double resolution, double origin_x,
                  double origin_y, double origin_yaw, std::vector<std::int8_t> data,
                  int occupied_threshold = 65, bool unknown_is_occupied = true);

    static OccupancyGrid loadMapYaml(const std::string& yaml_path);
    bool valid() const;
    bool contains(int gx, int gy) const;
    bool occupiedCell(int gx, int gy) const;
    bool unknownCell(int gx, int gy) const;
    bool occupiedWorld(double x, double y) const;
    bool occupiedNearWorld(double x, double y, double radius) const;
    std::pair<int, int> worldToGrid(double x, double y) const;
    std::pair<double, double> gridToWorld(int gx, int gy) const;
    void setDynamicObstacles(const std::vector<LaserPoint>& points, double inflation_radius);
    void clearDynamicObstacles();

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

private:
    int width_ = 0, height_ = 0;
    double resolution_ = 0.0, origin_x_ = 0.0, origin_y_ = 0.0, origin_yaw_ = 0.0;
    int occupied_threshold_ = 65;
    bool unknown_is_occupied_ = true;
    std::vector<std::int8_t> data_;
    std::vector<std::uint8_t> dynamic_;
};
} // namespace robot::navigation
